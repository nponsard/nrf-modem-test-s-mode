#![no_std]
#![no_main]
use embassy_nrf::pac::uicr::vals::{Hfxocnt, Hfxosrc};
use tinyrlibc::*;

use cortex_m::peripheral::NVIC;
use defmt::{debug, expect, info};
use embassy_executor::Spawner;
use embassy_nrf::gpio::{Level, Output, OutputDrive};
use embassy_nrf::{interrupt, pac};
use embassy_time::Timer;
use nrf_modem::SystemMode;
use nrf_modem::{ConnectionPreference, MemoryLayout};
use {defmt_rtt as _, panic_probe as _};

extern "C" {
    static __start_ipc: u8;
    static __end_ipc: u8;
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    // Initializing embassy_nrf has to come first because it assumes POWER and CLOCK at the secure address
    let embassy_peripherals = embassy_nrf::init(Default::default());
    let mut led = Output::new(embassy_peripherals.P0_00, Level::Low, OutputDrive::Standard);

    let clock = embassy_nrf::pac::CLOCK_S;
    clock.tasks_hfclkstart().write_value(1);
    let stat = clock.hfclkstat().read();
    defmt::info!("HFCLK Source: {}", stat.src().to_bits());
    defmt::info!("HFCLK State: {}", stat.state());

    let uicr = embassy_nrf::pac::UICR_S;
    let hfxocnt = uicr.hfxocnt().read().hfxocnt().to_bits();
    let hfxosrc = uicr.hfxosrc().read().hfxosrc().to_bits();

    // uicr.hfxosrc().read();

    // uicr.hfxocnt().write(|w| w.set_hfxocnt(Hfxocnt(251)));
    // uicr.hfxosrc().write(|w| w.set_hfxosrc(Hfxosrc::TCXO));

    defmt::info!("HFXO Count: {}", hfxocnt);
    defmt::info!("HFXO Source: {}", hfxosrc);

    fn configure_modem_non_secure() -> u32 {
        // The RAM memory space is divided into 32 regions of 8 KiB.
        // Set IPC RAM to nonsecure
        const SPU_REGION_SIZE: u32 = 0x2000; // 8kb
        const RAM_START: u32 = 0x2000_0000; // 256kb
        let ipc_start: u32 = unsafe { &__start_ipc as *const u8 } as u32;
        let ipc_reg_offset = (ipc_start - RAM_START) / SPU_REGION_SIZE;
        let ipc_reg_count =
            (unsafe { &__end_ipc as *const u8 } as u32 - ipc_start) / SPU_REGION_SIZE;
        let spu = embassy_nrf::pac::SPU;
        let range = ipc_reg_offset..(ipc_reg_offset + ipc_reg_count);
        debug!("marking region as non secure: {}", range);
        for i in range {
            spu.ramregion(i as usize).perm().write(|w| {
                w.set_execute(true);
                w.set_write(true);
                w.set_read(true);
                w.set_secattr(false);
                w.set_lock(false);
            })
        }

        // Set regulator access registers to nonsecure
        spu.periphid(4).perm().write(|w| w.set_secattr(false));
        // Set clock and power access registers to nonsecure
        spu.periphid(5).perm().write(|w| w.set_secattr(false));
        // Set IPC access register to nonsecure
        spu.periphid(42).perm().write(|w| w.set_secattr(false));
        ipc_start
    }
    let ipc_start = configure_modem_non_secure();
    // Interrupt Handler for LTE related hardware. Defer straight to the library.
    #[interrupt]
    #[allow(non_snake_case)]
    fn IPC() {
        nrf_modem::ipc_irq_handler();
    }

    let mut cp = cortex_m::Peripherals::take().expect("Failed to take Cortex-M peripherals");

    // Enable the modem interrupts
    unsafe {
        NVIC::unmask(pac::Interrupt::IPC);
        cp.NVIC.set_priority(pac::Interrupt::IPC, 0 << 5);
    }

    nrf_modem::init_with_custom_layout(
        SystemMode {
            lte_support: true,
            lte_psm_support: true,
            nbiot_support: false,
            gnss_support: false,
            preference: ConnectionPreference::None,
        },
        MemoryLayout {
            base_address: ipc_start,
            tx_area_size: 0x2000,
            rx_area_size: 0x2000,
            trace_area_size: 0x1000,
        },
    )
    .await
    .unwrap();

    let response = nrf_modem::send_at::<64>("AT+CGMI").await.unwrap();
    defmt::info!("Modem Manufacturer: {}", response.as_str());
    loop {
        led.set_high();
        defmt::info!("high");
        Timer::after_millis(500).await;
        led.set_low();
        defmt::info!("low");
        Timer::after_millis(1000).await;
    }
}
