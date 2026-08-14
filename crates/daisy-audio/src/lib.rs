#![no_std]

//! Audio I/O for the Daisy Seed: SAI1 + double-buffered DMA, with a block-based
//! callback that runs in the DMA half/complete interrupt (following the
//! stm32h7xx-hal `sai_dma_passthru` example). Two codecs are supported via a
//! cargo feature:
//!
//! * **default (Seed 1.1) — WM8731** over I2C2, I2S / 24-bit. SAI1 block A = RX
//!   master, block B = TX slave.
//! * **`seed3` — TI TAC5242**, HARDWARE-strapped (no I2C, no reset). SAI1
//!   block A = **TX master**, block B = **RX slave**, 32-bit MSB/left-justified,
//!   MCLK driven on PE2. Ports the hardware-verified daisy-embassy PR #80 config
//!   (`src/codec/tac5242.rs`): the codec self-configures + auto-powers from the
//!   ASI clocks, so the "driver" is SAI setup only, plus a 2 ms settle delay and
//!   an [right,left]→[left,right] word swap at the board boundary.
//!
//! Buffers live in D2-domain SRAM (`.sram_d2`) — AHB-reachable by DMA1 (DTCM is
//! not) and non-cacheable, so no cache maintenance is needed.
//!
//! STATUS: builds against the HAL and boots under Renode (XIP). The SAI kernel
//! clock (PLL3_P) + `&CoreClocks` come from the clock `freeze()` (the XIP app
//! obtains `CoreClocks` via `daisy_bsp::clocks::handoff`). The actual audio,
//! codec register/strap behaviour, and exact SAI frame timing must be validated
//! on hardware — Renode has no SAI codec/analog path (the SAI *model* only
//! exercises the DMA-request/data-movement protocol).

#[cfg(target_os = "none")]
pub use bare::{Audio, AudioCallback, Pins, BLOCK_SIZE};

#[cfg(target_os = "none")]
mod bare {
    use core::mem::MaybeUninit;

    use daisy_bsp::hal;
    use hal::dma::{
        self, dma::StreamsTuple, DBTransfer, MemoryToPeripheral, PeripheralToMemory, Transfer,
    };
    use hal::gpio::gpioe;
    use hal::pac::{self, interrupt, DMA1, SAI1};
    use hal::rcc::{rec, CoreClocks};
    use hal::sai::{
        self, I2SChanConfig, I2SClockStrobe, I2SDataSize, I2SDir, I2SSync, I2sUsers, SaiChannel,
        SaiDmaExt, SaiI2sExt,
    };
    use hal::time::Hertz;
    use hal::traits::i2s::FullDuplex;

    // --- WM8731-only imports (Seed 1.1 I2C codec) -------------------------
    #[cfg(not(feature = "seed3"))]
    use hal::hal::blocking::i2c::Write as _;
    #[cfg(not(feature = "seed3"))]
    use hal::i2c::I2c;
    #[cfg(not(feature = "seed3"))]
    use hal::pac::I2C2;

    /// Frames per processing block.
    pub const BLOCK_SIZE: usize = 48;
    /// Interleaved stereo samples per block (L,R,L,R…).
    const STEREO_BLOCK: usize = BLOCK_SIZE * 2;
    /// The DMA ring holds two blocks (double-buffer halves).
    const DMA_BUFFER_LEN: usize = STEREO_BLOCK * 2;

    const SAMPLE_RATE_HZ: u32 = 48_000;
    #[cfg(not(feature = "seed3"))]
    const WM8731_I2C_ADDR: u8 = 0x1A;

    /// Full-scale sample magnitude for u32 ↔ f32 conversion.
    #[cfg(feature = "seed3")]
    const SCALE: f32 = 2_147_483_648.0; // 2^31 — TAC5242 32-bit words
    #[cfg(not(feature = "seed3"))]
    const SCALE: f32 = 8_388_608.0; // 2^23 — WM8731 24-bit words

    /// User audio callback: `(input, output)`, each interleaved stereo of
    /// length `STEREO_BLOCK`. Runs in the DMA1_STR1 interrupt.
    pub type AudioCallback = fn(input: &[f32; STEREO_BLOCK], output: &mut [f32; STEREO_BLOCK]);

    #[link_section = ".sram_d2"]
    static mut TX_BUFFER: MaybeUninit<[u32; DMA_BUFFER_LEN]> = MaybeUninit::uninit();
    #[link_section = ".sram_d2"]
    static mut RX_BUFFER: MaybeUninit<[u32; DMA_BUFFER_LEN]> = MaybeUninit::uninit();

    // The RX (capture) SAI channel differs by codec: WM8731 records on the
    // master block A; the TAC5242 records on the slave block B.
    #[cfg(feature = "seed3")]
    type RxSaiChannel = sai::dma::ChannelB<SAI1>;
    #[cfg(not(feature = "seed3"))]
    type RxSaiChannel = sai::dma::ChannelA<SAI1>;

    type RxTransfer = Transfer<
        dma::dma::Stream1<DMA1>,
        RxSaiChannel,
        PeripheralToMemory,
        &'static mut [u32; DMA_BUFFER_LEN],
        DBTransfer,
    >;

    static mut RX_TRANSFER: Option<RxTransfer> = None;
    static mut CALLBACK: Option<AudioCallback> = None;

    /// SAI1 pins (all GPIOE). Take pins in their reset (`Analog`) state;
    /// `Audio::new` sets the SAI alternate funcs. Neither build needs a
    /// codec-reset GPIO: the WM8731 (Seed 1.1) has no reset line and is
    /// configured over I2C2 (SCL=PH4, SDA=PB11), and the TAC5242 (Seed 3) is
    /// hardware-strapped. (ref: `reference/libDaisy/src/daisy_seed.cpp`
    /// `DAISY_SEED_1_1` + `dev/codec_wm8731.h` — the PB11 reset was an AK4556
    /// artifact from the original Seed, and PB11 is actually I2C2 SDA.)
    pub struct Pins {
        pub mclk_a: gpioe::PE2<hal::gpio::Analog>,
        pub sck_a: gpioe::PE5<hal::gpio::Analog>,
        pub fs_a: gpioe::PE4<hal::gpio::Analog>,
        pub sd_a: gpioe::PE6<hal::gpio::Analog>,
        pub sd_b: gpioe::PE3<hal::gpio::Analog>,
    }

    /// The SAI1 audio interface. After `start()`, the callback runs in the DMA
    /// interrupt every `BLOCK_SIZE` frames.
    pub struct Audio {
        _sai: sai::Sai<SAI1, sai::I2S>,
    }

    // Shared: acquire the two DMA buffers as zeroed 'static mut slices.
    fn take_buffers() -> (
        &'static mut [u32; DMA_BUFFER_LEN],
        &'static mut [u32; DMA_BUFFER_LEN],
    ) {
        let tx_buffer = unsafe {
            (*core::ptr::addr_of_mut!(TX_BUFFER)).write([0; DMA_BUFFER_LEN]);
            (*core::ptr::addr_of_mut!(TX_BUFFER)).assume_init_mut()
        };
        let rx_buffer = unsafe {
            (*core::ptr::addr_of_mut!(RX_BUFFER)).write([0; DMA_BUFFER_LEN]);
            (*core::ptr::addr_of_mut!(RX_BUFFER)).assume_init_mut()
        };
        (tx_buffer, rx_buffer)
    }

    fn base_dma_config() -> dma::dma::DmaConfig {
        dma::dma::DmaConfig::default()
            .priority(dma::config::Priority::High)
            .memory_increment(true)
            .peripheral_increment(false)
            .circular_buffer(true)
            .fifo_enable(false)
    }

    impl Audio {
        /// Bring up the TAC5242 (Seed 3) + SAI1 + DMA. The codec is hardware-
        /// strapped — no I2C/register init and no reset pin — so this only
        /// configures and starts the SAI. `sai1_rec` must already be muxed to
        /// PLL3_P (48 kHz kernel clock); `clocks` is the frozen core clock config
        /// (recovered from the bootloader via `daisy_bsp::clocks::handoff`).
        #[cfg(feature = "seed3")]
        pub fn new(
            sai1: SAI1,
            dma1: DMA1,
            dma1_rec: rec::Dma1,
            sai1_rec: rec::Sai1,
            pins: Pins,
            clocks: &CoreClocks,
        ) -> Self {
            // TAC5242 data sheet: ≥ 2 ms between stable supplies/mode pins and
            // starting the ASI clocks. ~2.5 ms at 400 MHz.
            cortex_m::asm::delay(1_000_000);

            let streams = StreamsTuple::new(dma1, dma1_rec);
            let (tx_buffer, rx_buffer) = take_buffers();
            let base_config = base_dma_config();

            // Stream 0 = TX (memory → SAI block A / master). Stream 1 = RX
            // (SAI block B / slave → memory) with half + complete IRQs.
            let mut tx: Transfer<_, _, MemoryToPeripheral, _, _> = Transfer::init(
                streams.0,
                unsafe { pac::Peripherals::steal().SAI1.dma_ch_a() },
                tx_buffer,
                None,
                base_config,
            );
            let rx_config = base_config
                .transfer_complete_interrupt(true)
                .half_transfer_interrupt(true);
            let mut rx: Transfer<_, _, PeripheralToMemory, _, _> = Transfer::init(
                streams.1,
                unsafe { pac::Peripherals::steal().SAI1.dma_ch_b() },
                rx_buffer,
                None,
                rx_config,
            );

            // SAI1: block A = TX master, block B = RX slave (synchronous). The
            // HAL's default protocol is MSB (= MSB-first / left-justified), which
            // is exactly the TAC5242 frame; BITS_32 gives the 64-bit stereo frame.
            let tx_cfg = I2SChanConfig::new(I2SDir::Tx)
                .set_frame_sync_active_high(true)
                .set_clock_strobe(I2SClockStrobe::Falling);
            let rx_cfg = I2SChanConfig::new(I2SDir::Rx)
                .set_sync_type(I2SSync::Internal)
                .set_frame_sync_active_high(true)
                .set_clock_strobe(I2SClockStrobe::Rising);

            let sai_pins = (
                pins.mclk_a.into_alternate(),
                pins.sck_a.into_alternate(),
                pins.fs_a.into_alternate(),
                pins.sd_a.into_alternate(),
                Some(pins.sd_b.into_alternate()),
            );

            let mut sai = sai1.i2s_ch_a(
                sai_pins,
                Hertz::from_raw(SAMPLE_RATE_HZ),
                I2SDataSize::BITS_32,
                sai1_rec,
                clocks,
                I2sUsers::new(tx_cfg).add_slave(rx_cfg),
            );

            // Start the RX (slave, block B) DMA, then the TX (master, block A):
            // prime its FIFO from the zeroed buffer and enable — starting the
            // master transmitter also clocks the synchronous receiver.
            rx.start(|_| sai.enable_dma(SaiChannel::ChannelB));
            tx.start(|rb| {
                sai.enable_dma(SaiChannel::ChannelA);
                while rb.cha().sr.read().flvl().is_empty() {}
                sai.enable();
                let _ = sai.try_send(0, 0);
            });

            unsafe {
                RX_TRANSFER = Some(rx);
            }
            Audio { _sai: sai }
        }

        /// Bring up the WM8731 (Seed 1.1) codec + SAI1 + DMA. `sai1_rec` must be
        /// muxed to PLL3_P; `clocks` is the frozen core clock config.
        #[cfg(not(feature = "seed3"))]
        pub fn new(
            sai1: SAI1,
            dma1: DMA1,
            dma1_rec: rec::Dma1,
            sai1_rec: rec::Sai1,
            i2c2: I2c<I2C2>,
            pins: Pins,
            clocks: &CoreClocks,
        ) -> Self {
            init_wm8731(i2c2);

            let streams = StreamsTuple::new(dma1, dma1_rec);
            let (tx_buffer, rx_buffer) = take_buffers();
            let base_config = base_dma_config();

            // Stream 0 = TX (memory → SAI block B / slave). Stream 1 = RX
            // (SAI block A / master → memory) with half + complete IRQs.
            let mut tx: Transfer<_, _, MemoryToPeripheral, _, _> = Transfer::init(
                streams.0,
                unsafe { pac::Peripherals::steal().SAI1.dma_ch_b() },
                tx_buffer,
                None,
                base_config,
            );
            let rx_config = base_config
                .transfer_complete_interrupt(true)
                .half_transfer_interrupt(true);
            let mut rx: Transfer<_, _, PeripheralToMemory, _, _> = Transfer::init(
                streams.1,
                unsafe { pac::Peripherals::steal().SAI1.dma_ch_a() },
                rx_buffer,
                None,
                rx_config,
            );

            // SAI1: block A = master RX, block B = slave TX, 48 kHz, 24-bit.
            let tx_cfg = I2SChanConfig::new(I2SDir::Tx)
                .set_frame_sync_active_high(true)
                .set_clock_strobe(I2SClockStrobe::Falling);
            let rx_cfg = I2SChanConfig::new(I2SDir::Rx)
                .set_sync_type(I2SSync::Internal)
                .set_frame_sync_active_high(true)
                .set_clock_strobe(I2SClockStrobe::Rising);

            let sai_pins = (
                pins.mclk_a.into_alternate(),
                pins.sck_a.into_alternate(),
                pins.fs_a.into_alternate(),
                pins.sd_a.into_alternate(),
                Some(pins.sd_b.into_alternate()),
            );

            let mut sai = sai1.i2s_ch_a(
                sai_pins,
                Hertz::from_raw(SAMPLE_RATE_HZ),
                I2SDataSize::BITS_24,
                sai1_rec,
                clocks,
                I2sUsers::new(rx_cfg).add_slave(tx_cfg),
            );

            rx.start(|_| sai.enable_dma(SaiChannel::ChannelB));
            tx.start(|rb| {
                sai.enable_dma(SaiChannel::ChannelA);
                while rb.cha().sr.read().flvl().is_empty() {}
                sai.enable();
                let _ = sai.try_send(0, 0);
            });

            unsafe {
                RX_TRANSFER = Some(rx);
            }
            Audio { _sai: sai }
        }

        /// Install the processing callback and unmask the DMA interrupt.
        pub fn start(&mut self, callback: AudioCallback) {
            unsafe {
                CALLBACK = Some(callback);
                pac::NVIC::unmask(pac::Interrupt::DMA1_STR1);
            }
        }
    }

    /// DMA1 stream 1 (RX) half/complete: convert the freshly captured half to
    /// f32, run the callback, and write its output into the matching TX half.
    #[interrupt]
    fn DMA1_STR1() {
        let tx = unsafe { (*core::ptr::addr_of_mut!(TX_BUFFER)).assume_init_mut() };
        let rx = unsafe { (*core::ptr::addr_of_mut!(RX_BUFFER)).assume_init_mut() };

        let transfer = match unsafe { &mut *core::ptr::addr_of_mut!(RX_TRANSFER) } {
            Some(t) => t,
            None => return,
        };

        // Which half just completed → its slice offset (the DMA is filling the
        // other half now).
        let offset = if transfer.get_half_transfer_flag() {
            transfer.clear_half_transfer_interrupt();
            0
        } else if transfer.get_transfer_complete_flag() {
            transfer.clear_transfer_complete_interrupt();
            STEREO_BLOCK
        } else {
            return;
        };

        let mut input = [0.0f32; STEREO_BLOCK];
        let mut output = [0.0f32; STEREO_BLOCK];

        // Seed 3's TAC5242 presents receive words as [right, left]; swap at the
        // board boundary so the callback contract stays [left, right]. (Verified
        // empirically in daisy-embassy on a stock Daisy Pod.)
        #[cfg(feature = "seed3")]
        for frame in 0..BLOCK_SIZE {
            let l = offset + frame * 2;
            input[l] = (rx[l + 1] as i32) as f32 / SCALE;
            input[l + 1] = (rx[l] as i32) as f32 / SCALE;
        }
        #[cfg(not(feature = "seed3"))]
        for i in 0..STEREO_BLOCK {
            input[i] = (rx[offset + i] as i32) as f32 / SCALE;
        }

        match unsafe { &*core::ptr::addr_of!(CALLBACK) } {
            Some(cb) => cb(&input, &mut output),
            None => output = input, // default passthrough
        }

        #[cfg(feature = "seed3")]
        for frame in 0..BLOCK_SIZE {
            let l = offset + frame * 2;
            tx[l] = ((output[l + 1] * SCALE) as i32) as u32;
            tx[l + 1] = ((output[l] * SCALE) as i32) as u32;
        }
        #[cfg(not(feature = "seed3"))]
        for i in 0..STEREO_BLOCK {
            tx[offset + i] = ((output[i] * SCALE) as i32) as u32;
        }
    }

    /// WM8731 init over I2C2 for I2S / 24-bit / 48 kHz / **slave** with an
    /// external 12.288 MHz MCLK (256 × fs) supplied by the SAI. A control word
    /// is the 7-bit register INDEX in bits [15:9] + 9 data bits, packed into two
    /// bytes (`byte0 = index<<1 | data[8]`, `byte1 = data[7:0]`).
    ///
    /// Register values verified field-by-field against the WM8731 datasheet
    /// (`reference/WolfsonWM8731.pdf`, Tables 3–29). The ANALOG path + real
    /// audio still need hardware validation, but the register encodings are
    /// datasheet-correct. Powers down the internal oscillator, CLKOUT and the
    /// unused microphone (external-MCLK config, RM/datasheet p31/p44/p45).
    #[cfg(not(feature = "seed3"))]
    fn init_wm8731(mut i2c: I2c<I2C2>) {
        // The Seed 1.1 WM8731 has NO hardware reset line (unlike the AK4556 on
        // the original Seed, which used PB11) — it is configured purely over
        // I2C2 (SCL=PH4, SDA=PB11). R15 below is the software reset.

        // (register index, 9-bit value)
        let writes: [(u8, u16); 10] = [
            (0x0F, 0b0_0000_0000), // R15 reset
            (0x06, 0b0_0110_0010), // R6 power: up line-in/ADC/DAC/out; OSC+CLKOUT+MIC off
            (0x00, 0b0_0001_0111), // R0 L line-in: 0 dB, unmute
            (0x01, 0b0_0001_0111), // R1 R line-in: 0 dB, unmute
            (0x04, 0b0_0001_0010), // R4 analog: INSEL=line, BYPASS off, DACSEL on, MUTEMIC on
            (0x05, 0b0_0000_0000), // R5 digital: DAC un-muted, ADC HPF on, de-emph off
            (0x07, 0b0_0000_1010), // R7 interface: I2S (FORMAT=10), 24-bit (IWL=10), slave (MS=0)
            (0x08, 0b0_0000_0000), // R8 sampling: NORMAL, 256fs, SR=0000 → 48 kHz @ 12.288 MHz MCLK
            (0x09, 0b0_0000_0001), // R9 activate
            (0x06, 0b0_0110_0010), // R6 power: same as above (keep OSC/CLKOUT/MIC OFF — NOT 0)
        ];
        for (reg, val) in writes {
            let byte0 = (reg << 1) | ((val >> 8) as u8 & 1);
            let byte1 = (val & 0xFF) as u8;
            let _ = i2c.write(WM8731_I2C_ADDR, &[byte0, byte1]);
            cortex_m::asm::delay(48_000);
        }
    }
}
