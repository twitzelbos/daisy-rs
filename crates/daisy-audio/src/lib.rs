#![no_std]

//! Audio I/O for the Daisy Seed: SAI1 + double-buffered DMA, with a block-based
//! callback that runs in the DMA half/complete interrupt (following the
//! stm32h7xx-hal `sai_dma_passthru` example).
//!
//! # Codecs
//!
//! Four codecs across the Daisy generations are supported. The three **classic
//! Seeds are auto-detected at runtime** from the board-version straps (PD3/PD4),
//! exactly like libDaisy's `CheckBoardVersion`; the **Seed 3** is a separate
//! board selected at compile time with the `seed3` feature:
//!
//! | Board | Codec | Init | SAI dir (A/B) | Word |
//! |-------|-------|------|---------------|------|
//! | Seed v1 (default) | **AK4556** | reset pulse (PB11) | A=TX / B=RX | 24-bit |
//! | Seed 1.1 (PD3→gnd) | **WM8731** | I²C2 (PH4/PB11) | A=RX / B=TX | 24-bit |
//! | Seed 2 DFM (PD4→gnd) | **PCM3060** | de-emph pin (PB11) | A=TX / B=RX | 24-bit |
//! | Seed 3 (`seed3`) | **TAC5242** | hardware-strapped | A=TX / B=RX slave | 32-bit |
//!
//! The per-codec facts (word size, full-scale, SAI topology, channel order,
//! I²C program) live in the host-tested [`codec`] module; this crate's target
//! bring-up just consumes them. Only the WM8731 captures on data line A, which
//! flips the SAI master/slave + DMA-channel wiring — that is the one bit the
//! bring-up branches on.
//!
//! **Validation status:** AK4556 (Seed v1) and TAC5242 (Seed 3) are the codecs
//! we have hardware for. WM8731 and PCM3060 compile and are datasheet/libDaisy-
//! faithful but are **not hardware-validated**.
//!
//! Buffers live in D2-domain SRAM (`.sram_d2`) — AHB-reachable by DMA1 (DTCM is
//! not) and non-cacheable, so no cache maintenance is needed.
//!
//! STATUS: builds against the HAL and boots under Renode (XIP). The SAI kernel
//! clock (PLL3_P) + `&CoreClocks` come from the clock `freeze()` (the XIP app
//! obtains `CoreClocks` via `daisy_bsp::clocks::handoff`). Actual audio, codec
//! register/strap behaviour, and exact SAI frame timing must be validated on
//! hardware — Renode has no SAI codec/analog path.

pub mod codec;

#[cfg(target_os = "none")]
pub use bare::{Audio, AudioCallback, Pins, BLOCK_SIZE};

#[cfg(target_os = "none")]
mod bare {
    use core::mem::MaybeUninit;
    use core::sync::atomic::{AtomicBool, AtomicU32, Ordering};

    use crate::codec::Codec;

    use daisy_bsp::hal;
    use hal::dma::{
        self, dma::StreamsTuple, DBTransfer, MemoryToPeripheral, PeripheralToMemory, Transfer,
    };
    use hal::gpio::{gpioe, Alternate, Analog};
    use hal::pac::{self, interrupt, DMA1, SAI1};
    use hal::rcc::{rec, CoreClocks};
    use hal::sai::{
        self, I2SChanConfig, I2SClockStrobe, I2SDataSize, I2SDir, I2SSync, I2sUsers, SaiChannel,
        SaiDmaExt, SaiI2sExt,
    };
    use hal::time::Hertz;
    use hal::traits::i2s::FullDuplex;

    // --- Classic-Seed imports (runtime-detected AK4556 / WM8731 / PCM3060) ----
    #[cfg(not(feature = "seed3"))]
    use crate::codec::{wm8731_control_word, WM8731_I2C_ADDR, WM8731_INIT};
    #[cfg(not(feature = "seed3"))]
    use hal::gpio::{gpiob::PB11, gpiod::PD3, gpiod::PD4, gpioh::PH4};
    #[cfg(not(feature = "seed3"))]
    use hal::hal::blocking::i2c::Write as _;
    #[cfg(not(feature = "seed3"))]
    use hal::i2c::{I2c, I2cExt};
    #[cfg(not(feature = "seed3"))]
    use hal::pac::I2C2;

    /// Frames per processing block.
    pub const BLOCK_SIZE: usize = 48;
    /// Interleaved stereo samples per block (L,R,L,R…).
    const STEREO_BLOCK: usize = BLOCK_SIZE * 2;
    /// The DMA ring holds two blocks (double-buffer halves).
    const DMA_BUFFER_LEN: usize = STEREO_BLOCK * 2;

    const SAMPLE_RATE_HZ: u32 = 48_000;

    /// User audio callback: `(input, output)`, each interleaved stereo of
    /// length `STEREO_BLOCK`. Runs in the DMA1_STR1 interrupt.
    pub type AudioCallback = fn(input: &[f32; STEREO_BLOCK], output: &mut [f32; STEREO_BLOCK]);

    #[link_section = ".sram_d2"]
    static mut TX_BUFFER: MaybeUninit<[u32; DMA_BUFFER_LEN]> = MaybeUninit::uninit();
    #[link_section = ".sram_d2"]
    static mut RX_BUFFER: MaybeUninit<[u32; DMA_BUFFER_LEN]> = MaybeUninit::uninit();

    // Per-codec sample conversion, resolved at `Audio::new` and read in the ISR.
    // `SCALE_BITS` is the f32 full-scale as raw bits; `SWAP_LR` requests the
    // TAC5242's [right,left]→[left,right] channel swap.
    static SCALE_BITS: AtomicU32 = AtomicU32::new(0);
    static SWAP_LR: AtomicBool = AtomicBool::new(false);

    // The RX (capture) DMA transfer. Its SAI channel differs by codec: the
    // WM8731 records on the master block A, every other board on block B — a
    // *type-level* difference, so the stored transfer is an enum over the two.
    // ChannelA RX is used only by the WM8731 (A=RX) path, which the `seed3`
    // build excludes.
    #[cfg(not(feature = "seed3"))]
    type RxTransferA = Transfer<
        dma::dma::Stream1<DMA1>,
        sai::dma::ChannelA<SAI1>,
        PeripheralToMemory,
        &'static mut [u32; DMA_BUFFER_LEN],
        DBTransfer,
    >;
    type RxTransferB = Transfer<
        dma::dma::Stream1<DMA1>,
        sai::dma::ChannelB<SAI1>,
        PeripheralToMemory,
        &'static mut [u32; DMA_BUFFER_LEN],
        DBTransfer,
    >;

    enum RxTransfer {
        #[cfg(not(feature = "seed3"))]
        ChA(RxTransferA),
        ChB(RxTransferB),
    }

    impl RxTransfer {
        /// If a half/complete IRQ is pending, clear it and return the interleaved
        /// offset of the half that just filled (`0` or `STEREO_BLOCK`).
        fn drain(&mut self) -> Option<usize> {
            macro_rules! drain {
                ($t:expr) => {{
                    if $t.get_half_transfer_flag() {
                        $t.clear_half_transfer_interrupt();
                        Some(0)
                    } else if $t.get_transfer_complete_flag() {
                        $t.clear_transfer_complete_interrupt();
                        Some(STEREO_BLOCK)
                    } else {
                        None
                    }
                }};
            }
            match self {
                #[cfg(not(feature = "seed3"))]
                RxTransfer::ChA(t) => drain!(t),
                RxTransfer::ChB(t) => drain!(t),
            }
        }
    }

    static mut RX_TRANSFER: Option<RxTransfer> = None;
    static mut CALLBACK: Option<AudioCallback> = None;

    /// SAI1 pins (all GPIOE), plus — on the classic-Seed (non-`seed3`) build —
    /// the two board-version straps (PD3/PD4), the I²C2 SCL (PH4) and the
    /// overloaded codec-control pin (PB11: reset / SDA / de-emphasis, resolved
    /// after detection). Take pins in their reset (`Analog`) state; `Audio::new`
    /// sets the alternate funcs.
    pub struct Pins {
        pub mclk_a: gpioe::PE2<Analog>,
        pub sck_a: gpioe::PE5<Analog>,
        pub fs_a: gpioe::PE4<Analog>,
        pub sd_a: gpioe::PE6<Analog>,
        pub sd_b: gpioe::PE3<Analog>,
        #[cfg(not(feature = "seed3"))]
        pub pd3: PD3<Analog>,
        #[cfg(not(feature = "seed3"))]
        pub pd4: PD4<Analog>,
        #[cfg(not(feature = "seed3"))]
        pub scl: PH4<Analog>,
        #[cfg(not(feature = "seed3"))]
        pub ctrl: PB11<Analog>,
    }

    /// The SAI1 audio interface. After `start()`, the callback runs in the DMA
    /// interrupt every `BLOCK_SIZE` frames.
    pub struct Audio {
        _sai: sai::Sai<SAI1, sai::I2S>,
    }

    type Sai1I2s = sai::Sai<SAI1, sai::I2S>;
    /// The five SAI1 signal pins, already in alternate mode, in the order
    /// `i2s_ch_a` expects: `(MCLK_A, SCK_A, FS_A, SD_A, Some(SD_B))`.
    type SaiPins = (
        gpioe::PE2<Alternate<6>>,
        gpioe::PE5<Alternate<6>>,
        gpioe::PE4<Alternate<6>>,
        gpioe::PE6<Alternate<6>>,
        Option<gpioe::PE3<Alternate<6>>>,
    );

    /// Move the five SAI pins into alternate mode and tuple them.
    fn alt_sai_pins(
        mclk_a: gpioe::PE2<Analog>,
        sck_a: gpioe::PE5<Analog>,
        fs_a: gpioe::PE4<Analog>,
        sd_a: gpioe::PE6<Analog>,
        sd_b: gpioe::PE3<Analog>,
    ) -> SaiPins {
        (
            mclk_a.into_alternate(),
            sck_a.into_alternate(),
            fs_a.into_alternate(),
            sd_a.into_alternate(),
            Some(sd_b.into_alternate()),
        )
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

    /// Bring up the SAI1 in the **A = TX master / B = RX slave** topology
    /// (AK4556, PCM3060, TAC5242). RX lands on DMA/​SAI channel B.
    fn build_a_tx_b_rx(
        sai1: SAI1,
        dma1: DMA1,
        dma1_rec: rec::Dma1,
        sai1_rec: rec::Sai1,
        sai_pins: SaiPins,
        data_size: I2SDataSize,
        clocks: &CoreClocks,
    ) -> (Sai1I2s, RxTransfer) {
        let streams = StreamsTuple::new(dma1, dma1_rec);
        let (tx_buffer, rx_buffer) = take_buffers();
        let base_config = base_dma_config();

        // Stream 0 = TX (memory → block A / master). Stream 1 = RX
        // (block B / slave → memory) with half + complete IRQs.
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

        let tx_cfg = I2SChanConfig::new(I2SDir::Tx)
            .set_frame_sync_active_high(true)
            .set_clock_strobe(I2SClockStrobe::Falling);
        let rx_cfg = I2SChanConfig::new(I2SDir::Rx)
            .set_sync_type(I2SSync::Internal)
            .set_frame_sync_active_high(true)
            .set_clock_strobe(I2SClockStrobe::Rising);

        let mut sai = sai1.i2s_ch_a(
            sai_pins,
            Hertz::from_raw(SAMPLE_RATE_HZ),
            data_size,
            sai1_rec,
            clocks,
            I2sUsers::new(tx_cfg).add_slave(rx_cfg),
        );

        // Start RX (slave, block B), then TX (master, block A): prime its FIFO
        // from the zeroed buffer and enable — starting the master transmitter
        // also clocks the synchronous receiver.
        rx.start(|_| sai.enable_dma(SaiChannel::ChannelB));
        tx.start(|rb| {
            sai.enable_dma(SaiChannel::ChannelA);
            while rb.cha().sr.read().flvl().is_empty() {}
            sai.enable();
            let _ = sai.try_send(0, 0);
        });

        (sai, RxTransfer::ChB(rx))
    }

    /// Bring up the SAI1 in the **A = RX master / B = TX slave** topology
    /// (WM8731 only). RX lands on DMA/​SAI channel A. **Not HW-validated.**
    #[cfg(not(feature = "seed3"))]
    fn build_a_rx_b_tx(
        sai1: SAI1,
        dma1: DMA1,
        dma1_rec: rec::Dma1,
        sai1_rec: rec::Sai1,
        sai_pins: SaiPins,
        data_size: I2SDataSize,
        clocks: &CoreClocks,
    ) -> (Sai1I2s, RxTransfer) {
        let streams = StreamsTuple::new(dma1, dma1_rec);
        let (tx_buffer, rx_buffer) = take_buffers();
        let base_config = base_dma_config();

        // Stream 0 = TX (memory → block B / slave). Stream 1 = RX
        // (block A / master → memory) with half + complete IRQs.
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

        let rx_cfg = I2SChanConfig::new(I2SDir::Rx)
            .set_frame_sync_active_high(true)
            .set_clock_strobe(I2SClockStrobe::Rising);
        let tx_cfg = I2SChanConfig::new(I2SDir::Tx)
            .set_sync_type(I2SSync::Internal)
            .set_frame_sync_active_high(true)
            .set_clock_strobe(I2SClockStrobe::Falling);

        let mut sai = sai1.i2s_ch_a(
            sai_pins,
            Hertz::from_raw(SAMPLE_RATE_HZ),
            data_size,
            sai1_rec,
            clocks,
            I2sUsers::new(rx_cfg).add_slave(tx_cfg),
        );

        // Prime the TX (slave, block B) FIFO, then enable both DMAs and the SAI:
        // block A is the master receiver and generates the clock.
        tx.start(|rb| {
            sai.enable_dma(SaiChannel::ChannelB);
            while rb.chb().sr.read().flvl().is_empty() {}
        });
        rx.start(|_| sai.enable_dma(SaiChannel::ChannelA));
        sai.enable();
        let _ = sai.try_send(0, 0);

        (sai, RxTransfer::ChA(rx))
    }

    impl Audio {
        /// Bring up the Seed 3's TAC5242 + SAI1 + DMA. The codec is hardware-
        /// strapped (no I²C/reset), so this only configures and starts the SAI.
        /// `sai1_rec` must already be muxed to PLL3_P (48 kHz kernel clock);
        /// `clocks` is the frozen core-clock config recovered from the bootloader.
        #[cfg(feature = "seed3")]
        pub fn new(
            sai1: SAI1,
            dma1: DMA1,
            dma1_rec: rec::Dma1,
            sai1_rec: rec::Sai1,
            pins: Pins,
            clocks: &CoreClocks,
        ) -> Self {
            let codec = Codec::Tac5242;
            SCALE_BITS.store(codec.scale().to_bits(), Ordering::Relaxed);
            SWAP_LR.store(codec.swap_lr(), Ordering::Relaxed);

            // TAC5242 data sheet: ≥ 2 ms between stable supplies/mode pins and
            // starting the ASI clocks. ~2.5 ms at 400 MHz.
            cortex_m::asm::delay(1_000_000);

            let sai_pins = alt_sai_pins(pins.mclk_a, pins.sck_a, pins.fs_a, pins.sd_a, pins.sd_b);
            let (sai, rx) = build_a_tx_b_rx(
                sai1,
                dma1,
                dma1_rec,
                sai1_rec,
                sai_pins,
                I2SDataSize::BITS_32,
                clocks,
            );
            unsafe {
                RX_TRANSFER = Some(rx);
            }
            Audio { _sai: sai }
        }

        /// Bring up a **classic Daisy Seed** (v1 / 1.1 / 2 DFM): detect the
        /// codec from the PD3/PD4 straps, run its init (AK4556 reset pulse /
        /// WM8731 I²C program / PCM3060 de-emphasis), then configure + start the
        /// SAI in the topology that codec needs. `sai1_rec` must be muxed to
        /// PLL3_P; `clocks` is the frozen core-clock config.
        #[cfg(not(feature = "seed3"))]
        #[allow(clippy::too_many_arguments)]
        pub fn new(
            sai1: SAI1,
            dma1: DMA1,
            dma1_rec: rec::Dma1,
            sai1_rec: rec::Sai1,
            i2c2: I2C2,
            i2c2_rec: rec::I2c2,
            pins: Pins,
            clocks: &CoreClocks,
        ) -> Self {
            // Detect the board/codec from the two version straps (active-low,
            // pulled up): PD3 low ⇒ Seed 1.1, PD4 low ⇒ Seed 2 DFM, else Seed v1.
            let pd3 = pins.pd3.into_pull_up_input();
            let pd4 = pins.pd4.into_pull_up_input();
            cortex_m::asm::delay(10_000); // let the pull-ups settle
            let codec = Codec::from_straps(pd3.is_low(), pd4.is_low());
            SCALE_BITS.store(codec.scale().to_bits(), Ordering::Relaxed);
            SWAP_LR.store(codec.swap_lr(), Ordering::Relaxed);

            // Codec-specific init, consuming the overloaded PB11 control pin
            // (and, for the WM8731, PH4 + I²C2). Output pins keep their ODR bit
            // in hardware after the token is dropped.
            match codec {
                Codec::Wm8731 => {
                    // PH4 = I2C2_SCL, PB11 = I2C2_SDA, both AF4 open-drain.
                    let i2c = i2c2.i2c(
                        (
                            pins.scl.into_alternate_open_drain::<4>(),
                            pins.ctrl.into_alternate_open_drain::<4>(),
                        ),
                        Hertz::from_raw(400_000),
                        i2c2_rec,
                        clocks,
                    );
                    init_wm8731(i2c);
                }
                Codec::Ak4556 => {
                    // Reset pulse: drive HIGH → LOW → HIGH, ~1 ms per level. The
                    // explicit initial high guarantees the codec sees a clean
                    // falling edge on PDN before it's released — the sequence
                    // HW-validated on the original Seed (was PR #54). The pin
                    // stays high after this fn returns (dropping the token leaves
                    // the ODR bit).
                    let mut reset = pins.ctrl.into_push_pull_output();
                    reset.set_high();
                    cortex_m::asm::delay(480_000); // ~1 ms
                    reset.set_low();
                    cortex_m::asm::delay(480_000);
                    reset.set_high();
                    cortex_m::asm::delay(480_000);
                }
                Codec::Pcm3060 => {
                    // Hold the de-emphasis-disable pin low for the app's lifetime.
                    let mut deemp = pins.ctrl.into_push_pull_output();
                    deemp.set_low();
                }
                Codec::Tac5242 => {}
            }

            let data_size = if codec.bits() == 32 {
                I2SDataSize::BITS_32
            } else {
                I2SDataSize::BITS_24
            };
            let sai_pins = alt_sai_pins(pins.mclk_a, pins.sck_a, pins.fs_a, pins.sd_a, pins.sd_b);
            let (sai, rx) = if codec.a_is_rx() {
                build_a_rx_b_tx(sai1, dma1, dma1_rec, sai1_rec, sai_pins, data_size, clocks)
            } else {
                build_a_tx_b_rx(sai1, dma1, dma1_rec, sai1_rec, sai_pins, data_size, clocks)
            };
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

    /// WM8731 init over I²C2 for I²S / 24-bit / 48 kHz / **slave** with an
    /// external 12.288 MHz MCLK (256 × fs) from the SAI. Register program +
    /// control-word packing live in [`crate::codec`] (datasheet-verified).
    /// **Not hardware-validated.**
    #[cfg(not(feature = "seed3"))]
    fn init_wm8731(mut i2c: I2c<I2C2>) {
        for (reg, val) in WM8731_INIT {
            let word = wm8731_control_word(reg, val);
            let _ = i2c.write(WM8731_I2C_ADDR, &word);
            cortex_m::asm::delay(48_000);
        }
    }

    /// DMA1 stream 1 (RX) half/complete: convert the freshly captured half to
    /// f32, run the callback, and write its output into the matching TX half.
    #[interrupt]
    fn DMA1_STR1() {
        let tx = unsafe { (*core::ptr::addr_of_mut!(TX_BUFFER)).assume_init_mut() };
        let rx = unsafe { (*core::ptr::addr_of_mut!(RX_BUFFER)).assume_init_mut() };

        let offset = match unsafe { &mut *core::ptr::addr_of_mut!(RX_TRANSFER) } {
            Some(t) => match t.drain() {
                Some(o) => o,
                None => return,
            },
            None => return,
        };

        let scale = f32::from_bits(SCALE_BITS.load(Ordering::Relaxed));
        let swap = SWAP_LR.load(Ordering::Relaxed);

        let mut input = [0.0f32; STEREO_BLOCK];
        let mut output = [0.0f32; STEREO_BLOCK];

        // `offset` indexes the DMA ring (`DMA_BUFFER_LEN`); the callback buffers
        // are `STEREO_BLOCK`, indexed from 0. The TAC5242 swaps L/R per frame.
        if swap {
            for frame in 0..BLOCK_SIZE {
                let r = offset + frame * 2;
                let w = frame * 2;
                input[w] = (rx[r + 1] as i32) as f32 / scale;
                input[w + 1] = (rx[r] as i32) as f32 / scale;
            }
        } else {
            for i in 0..STEREO_BLOCK {
                input[i] = (rx[offset + i] as i32) as f32 / scale;
            }
        }

        match unsafe { &*core::ptr::addr_of!(CALLBACK) } {
            Some(cb) => cb(&input, &mut output),
            None => output = input, // default passthrough
        }

        if swap {
            for frame in 0..BLOCK_SIZE {
                let r = offset + frame * 2;
                let w = frame * 2;
                tx[r] = ((output[w + 1] * scale) as i32) as u32;
                tx[r + 1] = ((output[w] * scale) as i32) as u32;
            }
        } else {
            for i in 0..STEREO_BLOCK {
                tx[offset + i] = ((output[i] * scale) as i32) as u32;
            }
        }
    }
}
