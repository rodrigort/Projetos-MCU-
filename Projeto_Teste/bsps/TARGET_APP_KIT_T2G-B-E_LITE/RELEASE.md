# KIT_T2G-B-E_LITE BSP Release Notes
The KIT_T2G-B-E_LITE, a 100-pin evaluation board is based on the TRAVEO™ T2G Body Entry family of devices. TRAVEO™ T2G Body Entry MCU is designed for  automotive and industrial applications. The evaluation board carries a TRAVEO™ T2G Body Entry microcontroller,and headers compatible with Arduino for interfacing Arduino shields. In addition, the board features an on-board programmer/debugger (KitProg3),  CAN FD transceiver, a Shield2Go connector interface and MikroBUS connector interface, three user LEDs, one potentiometer,  and two user push buttons. The board supports operating voltages from 3.3 V to 5.0 V for TRAVEO™ T2G Body Entry device.     
**Note:**
Some additional setups will allow you to use more code examples than those shown on the next page. Click 
[here ](https://github.com/Infineon/TRAVEO_T2G_code_examples#how-to-setup)
for setup instructions.

NOTE: BSPs are versioned by family. This means that version 1.2.0 of any BSP in a family (eg: PSoC™ 6) will have the same software maturity level. However, not all updates are necessarily applicable for each BSP in the family so not all version numbers will exist for each board. Additionally, new BSPs may not start at version 1.0.0. In the event of adding a common feature across all BSPs, the libraries are assigned the same version number. For example if BSP_A is at v1.3.0 and BSP_B is at v1.2.0, the event will trigger a version update to v1.4.0 for both BSP_A and BSP_B. This allows the common feature to be tracked in a consistent way.

### What's Included?
The KIT_T2G-B-E_LITE library includes the following:
* BSP specific makefile to configure the build process for the board
* cybsp.c/h files to initialize the board and any system peripherals
* cybsp_types.h file describing basic board setup
* CM4 Linker script & startup code for GCC, IAR, and ARM toolchains
* CM0+ Linker script & startup code for GCC, IAR, and ARM toolchains
* Configurator design files (and generated code) to setup board specific peripherals
* .lib file references for all dependent libraries
* API documentation

### What Changed?
#### v4.3.1
Updated the README file for CY8CEVAL-062S2-MUR-43439M2.
#### v4.3.0
* Update companion device pin configuration to align with requirements of MUR-43439 and LAI-43439 M.2 module.
* Updated the README file for CY8CEVAL-062S2, CY8CEVAL-062S2-LAI-43439M2, CY8CEVAL-062S2-LAI-4373M2, CY8CEVAL-062S2-MUR-4373M2, CY8CEVAL-062S2-MUR-4373EM2, and CY8CEVAL-062S2-CYW43022CUB BSPs to remove the M.2 radio module detail from kit contents section.
#### v4.2.1
* Added memory-analyzer configuration support for CY8CKIT-062S4.
#### v4.2.0
* Updated linker scripts and startup code to align with mtb-pdl-cat1 v3.4.0
* Added functionality to enable BSP Assistant chip flow
* Added capabilities to match BSPS created by BSP Assistant chip flow
#### v4.1.0
* Add macro `CYBSP_USER_BTN_DRIVE` indicating the drive mode that should be used for user buttons
* PSoC 64 boards: Fix cybsp_init not recognizing that a prebuilt CM0+ image is in use when using TFM.
#### v4.0.0
Note: This revision is only compatible with ModusToolbox Tools 3.0 and newer.
* Removed default dependency on CAPSENSE™ middleware. The library manager can be used to add this dependency if desired.
* Updated recipe-make, core-make, and PDL to new major versions
* Regenerated code with Configurators from ModusToolbox™ v3.0.0
* Renamed top level board makefile to bsp.mk
* Removed version.xml file in favor of new props.json
#### v3.1.0
* Added optional macro CYBSP_CUSTOM_SYSCLK_PM_CALLBACK to allow overriding default clock power management behavior.
* Enable AIROC™ BLE stack for MCUs with an integrated BLE radio
#### v3.0.0
* Updated to HAL dependency to v2.0.0
* Updated CAPSENSE™ dependency to v3.0.0
* Regenerated code with Configurators from ModusToolbox™ v2.4.0
#### v2.3.0
* Add new connectivity components for easier board customization
* Simplify BT configuration settings for boards that support it
* Minor branding updates
#### v2.2.0
* Updated PSoC™ 64 linker sections to match secure policy settings
* Minor documentation updates
#### v2.1.0
* Added component CAT1 to all boards
* Added new components for connectivity chips
* Added BT configuration settings for boards that support it
* Minor documentation updates
#### v2.0.1
* Minor update to better handle when to include the SCL library in the build
#### v2.0.0
* Updated design files and GeneratedSource with ModusToolbox™ 2.2 release
* Migrated pin definitions into design.modus file
* Updated clock frequencies to 100 MHz (fast) / 50 MHz (slow)
* Updated MPNs on some boards to non-obsolete parts
* Switched psoc6pdl dependency to new mtb-pdl
* Switched psoc6hal dependency to new mtb-hal
* Switched psoc6make dependency to new core-make & recipe-make-cat1a
NOTE: This version requires ModusToolbox™ tools 2.2 or later. This version is not backwards compatible with 1.X versions. Additional manual steps must be taken to successfully update a design using a 1.x version of the BSP to this version.
#### v1.3.0
* Minor update for documentation & branding
* Updated design files to use latest personality files
* Initialize VDDA voltage if set in configurator
NOTE: This requires psoc6hal 1.3.0 or later
#### v1.2.1
* Added 43012/4343W/43438 component to appropriate BSPs
* Added multi-image policy for secure (064) BSPs
#### v1.2.0
* Standardize version numbering for all boards in a family
* Moved UDB SDIO implementation into its own library udb-sdio-whd library
* Added call to setup HAL SysPM driver (requires HAL 1.2.0 or later)
* Updated documentation
NOTE: This requires psoc6hal 1.2.0 or later
#### v1.1.0
* Updated linker scripts and startup code for the CM0+ and CM4 cores. The files are now in core specific directories.
* Minor updates to avoid potential warnings on some toolchains
#### v1.0.1
* Added pin references for the board's J2 Header (for appropriate boards)
#### v1.0.0
* Initial release

### Supported Software and Tools
This version of the KIT_T2G-B-E_LITE BSP was validated for compatibility with the following Software and Tools:

| Software and Tools                        | Version |
| :---                                      | :----:  |
| ModusToolbox™ Software Environment        | 3.1.0   |
| GCC Compiler                              | 12.2.1  |
| IAR Compiler                              | 9.40.2  |
| ARM Compiler                              | 6.16    |

Minimum required ModusToolbox™ Software Environment: v3.0.0

### More information
* [KIT_T2G-B-E_LITE BSP API Reference Manual][api]
* [KIT_T2G-B-E_LITE Documentation](https://www.infineon.com/KIT_T2G-B-E_LITE)
* [Cypress Semiconductor, an Infineon Technologies Company](http://www.cypress.com)
* [Infineon GitHub](https://github.com/infineon)
* [ModusToolbox™](https://www.cypress.com/products/modustoolbox-software-environment)

[api]: https://infineon.github.io/TARGET_KIT_T2G-B-E_LITE/html/modules.html

---
© Cypress Semiconductor Corporation (an Infineon company) or an affiliate of Cypress Semiconductor Corporation, 2019-2022.