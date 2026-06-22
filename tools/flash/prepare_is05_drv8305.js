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
 * runs. On success the target is left running and the user drives the actual identification
 * (flagRunIdentAndOnLine, speed ramp, convergence) in the normal CCS watch-window flow.
 *
 * Verification baked in (ENFORCED, not just printed): the script hard-fails -- pulls EN_GATE back
 * low, sets flagEnableSys=0, leaves the target HALTED, and exits nonzero -- unless every check
 * passes: parked at the dead-wait (halHandle valid, flagEnableSys==0), EN_GATE readback high,
 * flagEnableSys latched to 1, offset cal completed (flagEnableOffsetCalc==0), no latched fault
 * (faultUse.all==0), and a sane bus (VdcBus_V > 5 V). Only then is the target left running for ID.
 * Rationale: with the gate OFF the DRV8305 CSAs float and the (correctly-routed) CMPSS trips ->
 * faultUse=16; after EN_GATE the CSAs wake and faultUse drops to 0 at zero current -- so faultUse=0
 * is positive proof the gate is enabled and current sense is live, and a nonzero value must block ID.
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
function num(s,e){ try { return Number(e.evaluate(s)); } catch(err){ return NaN; } }
function set(s,v,e){ try { e.evaluate(s+"="+v); return true; } catch(err){ return false; } }

var ccxml=arguments[0], out=arguments[1];
var VBUS_MIN = 5.0;   // sane lower bound: bus present and the ADC is reading something real

var env=ScriptingEnvironment.instance(); env.setScriptTimeout(60000);
var server=env.getServer("DebugServer.1"); server.setConfig(ccxml);
var s=server.openSession("*","C28xx_CPU1");
s.target.connect(); s.memory.loadProgram(out);
var e=s.expression;

// On ANY failed precondition: drive the board back to a safe state (EN_GATE low via GPBCLEAR,
// flagEnableSys=0), leave the target HALTED (so the user cannot proceed into ID), and terminate.
// "Board ready" is printed only when every check below passes.
function fail(why){
    p("");
    p("!!!!!!!!!!!! is05 bring-up FAILED !!!!!!!!!!!!");
    p("  reason: " + why);
    s.memory.writeData(Memory.Page.DATA, 0x7F0C, 0x80, 16);  // GPBCLEAR bit7 -> EN_GATE low (gate off)
    set("motorVars.flagEnableSys", "0", e);                  // stop the bg loop from driving
    p("  -> EN_GATE pulled low, flagEnableSys=0, target left HALTED. Do NOT run is05 until fixed.");
    p("=============================================");
    try { s.target.disconnect(); } catch(x){}
    server.stop(); s.terminate();
    java.lang.System.exit(1);
}

p("");
p("============ is05 DRV8305 bring-up ============");
// 1) run init; the lab reaches "while(flagEnableSys==false)" within tens of ms.
s.target.runAsynch(); Thread.sleep(1200); s.target.halt();
var es0 = num("motorVars.flagEnableSys",e), hh = num("halHandle",e);
p("at dead-wait: flagEnableSys=" + es0 + " (expect 0)  halHandle=" + hh + " (expect nonzero)");
if(!(hh > 0))   fail("halHandle invalid -- HAL_init did not complete (not at the dead-wait).");
if(es0 !== 0)   fail("flagEnableSys != 0 -- target is not parked at the lab's enable-sys wait.");

// 2) assert EN_GATE directly (the lab skips HAL_enableDRV on DRV8305_SPI, and that function is
// dead-stripped from the binary; GpioDataRegs bitfields are not resolvable in DSS either).
// BOARD_GATE_ENABLE_GPIO=39 (active-high) = GPIO port B bit 7. GPBSET = GPIODATA_BASE(0x7F00) +
// GPIO_O_GPBSET(0xA) = 0x7F0A; writing 0x80 sets GPIO39 high. The DRV8305 wakes with its CSAs
// active on power-on defaults (the SDK SPI config only tweaks VDS level / dead-time / CSA gain).
p(">>> asserting EN_GATE (GPIO39 high, GPBSET<-0x80) to wake the DRV8305 ...");
s.memory.writeData(Memory.Page.DATA, 0x7F0A, 0x80, 16);   // GPBSET low word, bit7 = GPIO39
s.target.runAsynch(); Thread.sleep(50); s.target.halt();  // ~ms for DRV8305 wake
var gpbdat = s.memory.readData(Memory.Page.DATA, 0x7F08, 16, 1, false); // GPBDAT low word
var g39 = ((gpbdat[0] & 0x80) != 0) ? 1 : 0;
p("    EN_GATE: GPBDAT bit7 (GPIO39) = " + g39 + " (expect 1)");
if(g39 !== 1)   fail("EN_GATE readback low -- GPIO39 did not assert; gate driver not enabled.");

// 3) start offset cal: flagEnableSys=true (also validates the CMPSS fix on the lab path).
if(!set("motorVars.flagEnableSys", "1", e)) fail("could not set flagEnableSys=1.");
if(num("motorVars.flagEnableSys",e) !== 1)  fail("flagEnableSys did not latch to 1.");
p(">>> flagEnableSys=1; running offset cal ~3.5s ...");
s.target.runAsynch(); Thread.sleep(3500); s.target.halt();

var oc = num("motorVars.flagEnableOffsetCalc",e);
var fu = num("motorVars.faultUse.all",e);
var vb = num("motorVars.VdcBus_V",e);
p("post-cal: flagEnableOffsetCalc=" + oc + " (expect 0)  faultUse.all=" + fu +
  " (expect 0)  VdcBus_V=" + vb + " (expect > " + VBUS_MIN + ")");
if(oc !== 0)        fail("offset cal did not complete (flagEnableOffsetCalc still 1).");
if(fu !== 0)        fail("faultUse.all=" + fu + " -- a fault is latched (e.g. over-current / bus). Power stage not healthy.");
if(!(vb > VBUS_MIN)) fail("VdcBus_V=" + vb + " below " + VBUS_MIN + " V -- bus not present or voltage sense wrong.");

// All checks passed: gate enabled, cal done, no fault, bus sane. Leave the target running.
p("");
p(">>> READY: gate enabled, offset cal done, no fault, VdcBus_V=" + vb + ".");
p("    Continue motor ID in CCS: set the usual is05 run flags and watch convergence.");
p("==============================================");
s.target.runAsynch(); s.target.disconnect(); server.stop(); s.terminate();
