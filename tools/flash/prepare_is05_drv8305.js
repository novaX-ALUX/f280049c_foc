/*
 * prepare_is05_drv8305.js - bring up the SDK is05_motor_id lab on launchxl with the DRV8305 gate
 * driver actually enabled, then leave the target ready for the normal is05 identification flow.
 *
 * Usage: dss.sh tools/flash/prepare_is05_drv8305.js <ccxml> <is05_motor_id.out>
 *
 * Why this exists: the SDK lab only calls HAL_enableDRV() under #ifdef DRV8320_SPI
 * (is05_motor_id.c:426), but launchxl builds with DRV8305_SPI, so the lab never enables the gate
 * driver -> the power stage is dead and motor ID would read zero current / garbage. We must NOT
 * edit the vendor SDK source. HAL_enableDRV() is also dead-stripped from the is05 binary (nothing
 * references it), so we cannot DSS-call it. Instead we run the lab to its
 * "while(flagEnableSys == false)" wait (is05_motor_id.c:476), halt, assert EN_GATE by writing the
 * GPIO register directly (GPBSET<-0x80 -> GPIO39 high), then set flagEnableSys=true so offset cal
 * runs. The actual identification (flagRunIdentAndOnLine, speed ramp, convergence) is then driven
 * by the user in the normal CCS watch-window flow against the still-running target.
 *
 * Verification baked in: with the gate OFF the DRV8305 CSAs float and the (correctly-routed) CMPSS
 * trips -> faultUse=16; after EN_GATE the CSAs wake and faultUse drops to 0 at zero current. So a
 * post-cal faultUse=0 is positive proof the gate is enabled and current sense is live.
 *
 * Caveat: this only asserts EN_GATE; it does NOT run the DRV8305 SPI configure (VDS level / dead
 * time / CSA gain), so the device runs on power-on defaults (CSA gain 10 V/V, matching the 47.14 A
 * scaling). Fine for low-current motor ID bring-up; a production path should configure it over SPI.
 *
 * Safety: run with a current-limited supply and NO motor (or a known motor for ID). Enabling the
 * gate with 24 V bus + offset-cal 50%-duty PWM and no motor is the same condition product_main
 * already runs through; with the CMPSS routing fix, moduleOverCurrent stays 0 at zero current.
 */
importPackage(Packages.com.ti.debug.engine.scripting);
importPackage(Packages.com.ti.ccstudio.scripting.environment);
importPackage(Packages.java.lang);
function p(s){ System.out.println(s); }
function ev(s,e){ try { return e.evaluate(s)+""; } catch(err){ return "??"; } }
function set(s,v,e){ try { e.evaluate(s+"="+v); return true; } catch(err){ p("  set FAILED: "+s); return false; } }

var ccxml=arguments[0], out=arguments[1];
var env=ScriptingEnvironment.instance(); env.setScriptTimeout(60000);
var server=env.getServer("DebugServer.1"); server.setConfig(ccxml);
var s=server.openSession("*","C28xx_CPU1");
s.target.connect(); s.memory.loadProgram(out);
var e=s.expression;

p("");
p("============ is05 DRV8305 bring-up ============");
// 1) run init; the lab reaches "while(flagEnableSys==false)" within tens of ms.
s.target.runAsynch(); Thread.sleep(1200); s.target.halt();
p("at dead-wait: flagEnableSys=" + ev("motorVars.flagEnableSys",e) +
  " (expect 0)  halHandle=" + ev("halHandle",e) + " (expect nonzero -> HAL_init done)");

// 2) enable the DRV8305 gate driver over the debugger (the lab skips this on DRV8305_SPI).
// HAL_enableDRV() is dead-stripped from the is05 binary (its only call site is under the inactive
// #ifdef DRV8320_SPI), and the driverlib build exposes no GpioDataRegs bitfields to DSS. So assert
// EN_GATE by writing the GPIO register directly: BOARD_GATE_ENABLE_GPIO=39 (active-high) is GPIO
// port B bit 7. GPBSET = GPIODATA_BASE(0x7F00) + GPIO_O_GPBSET(0xA) = 0x7F0A; writing 0x80 sets
// GPIO39 high. The DRV8305 wakes with its CSAs active on default settings (the SDK SPI config only
// tweaks VDS level / dead-time / CSA gain).
p(">>> asserting EN_GATE (GPIO39 high, GPBSET<-0x80) to wake the DRV8305 ...");
s.memory.writeData(Memory.Page.DATA, 0x7F0A, 0x80, 16);   // GPBSET low word, bit7 = GPIO39
s.target.runAsynch(); Thread.sleep(50); s.target.halt();  // ~ms for DRV8305 wake
var gpbdat = s.memory.readData(Memory.Page.DATA, 0x7F08, 16, 1, false); // GPBDAT low word
p("    EN_GATE: GPBDAT bit7 (GPIO39) = " + (((gpbdat[0] & 0x80) != 0) ? 1 : 0) + " (expect 1)");

// 3) read back: bus voltage (ISR updates adcData during the wait) + no fault from the enable.
p("post-enable: adcData.dcBus_V=" + ev("adcData.dcBus_V",e) + " (expect ~bus volts)" +
  "  faultNow.all=" + ev("motorVars.faultNow.all",e) + " (expect 0)");

// 4) let offset cal run (also validates the CMPSS fix on the lab path): flagEnableSys=true.
set("motorVars.flagEnableSys", "1", e);
p(">>> flagEnableSys=1; running offset cal ~3.5s ...");
s.target.runAsynch(); Thread.sleep(3500); s.target.halt();

p("post-cal: flagEnableOffsetCalc=" + ev("motorVars.flagEnableOffsetCalc",e) + " (expect 0)" +
  "  faultUse.all=" + ev("motorVars.faultUse.all",e) + " (expect 0 -> no false OC)" +
  "  VdcBus_V=" + ev("motorVars.VdcBus_V",e) + " (expect ~bus volts)");
p("=> if faultUse=0 and VdcBus_V looks right, the gate is enabled and the board is ready.");
p("   Continue motor ID in CCS: set the usual is05 run flags and watch convergence.");
p("==============================================");

// leave the target running for the interactive identification flow.
s.target.runAsynch(); s.target.disconnect(); server.stop(); s.terminate();
