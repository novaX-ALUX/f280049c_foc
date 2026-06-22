/*
 * prepare_drv8305_gate.js - enable the DRV8305 gate driver for ANY launchxl sensorless_foc SDK lab,
 * over the debugger, with enforced readiness checks. Lab-agnostic; use for is02/is03/is04/is05/...
 *
 * Usage: dss.sh tools/flash/prepare_drv8305_gate.js <ccxml> <lab.out>
 *
 * Why: every SDK sensorless lab that drives the power stage calls HAL_enableDRV() only under
 * #ifdef DRV8320_SPI (e.g. is05_motor_id.c:426), but launchxl builds with DRV8305_SPI, so the lab
 * never enables the gate driver -> the power stage is dead. HAL_enableDRV() is also dead-stripped
 * from the binaries (nothing references it), so it cannot be DSS-called. We must NOT edit the
 * vendor SDK source, so instead: run the lab to its "while(flagEnableSys==false)" wait, assert
 * EN_GATE by writing the GPIO register directly, then set flagEnableSys=true so offset cal runs.
 *
 * Scope (deliberately conservative): this ONLY brings the gate up and verifies the power stage is
 * healthy at zero current. It does NOT make any lab-specific calibration/identification decision --
 * the user drives the lab's own flow (offset/gain trim, motor ID, open-loop spin, ...) from CCS
 * against the still-running target after this passes.
 *
 * EN_GATE detail: BOARD_GATE_ENABLE_GPIO=39 (active-high) = GPIO port B bit 7. GPBSET =
 * GPIODATA_BASE(0x7F00) + GPIO_O_GPBSET(0xA) = 0x7F0A (write 0x80 -> high); GPBCLEAR = +0xC = 0x7F0C
 * (write 0x80 -> low); GPBDAT = +0x8 = 0x7F08 (read bit 7). The DRV8305 wakes with its CSAs active
 * on power-on defaults (the SDK SPI config only tweaks VDS level / dead-time / CSA gain; the default
 * CSA gain 10 V/V matches the 47.14 A scaling -- fine for low-current bring-up / ID).
 *
 * Enforcement (ENFORCED, not just printed): hard-fails -- pulls EN_GATE low, sets flagEnableSys=0,
 * leaves the target HALTED, exits 1 -- unless every check passes: parked at the dead-wait
 * (halHandle valid, flagEnableSys==0), EN_GATE readback high, flagEnableSys latched, offset cal
 * done (flagEnableOffsetCalc==0), no latched fault (faultUse.all==0), sane bus (VdcBus_V > 5 V).
 * Rationale: gate OFF -> the DRV8305 CSAs float and the (correctly-routed) CMPSS trips ->
 * faultUse=16; gate ON -> CSAs wake and faultUse drops to 0 at zero current. So faultUse=0 is
 * positive proof the gate is enabled and current sense is live; a nonzero value must block the lab.
 *
 * Safety: current-limited supply; no prop / a known safe load. Enabling the gate with bus applied +
 * offset-cal 50%-duty PWM is the same condition product_main already runs through.
 */
importPackage(Packages.com.ti.debug.engine.scripting);
importPackage(Packages.com.ti.ccstudio.scripting.environment);
importPackage(Packages.java.lang);
function p(s){ System.out.println(s); }
function num(s,e){ try { return Number(e.evaluate(s)); } catch(err){ return NaN; } }
function set(s,v,e){ try { e.evaluate(s+"="+v); return true; } catch(err){ return false; } }
// DSS expression.evaluate() TRUNCATES float32 to integer (e.g. 25.34 -> 25). Read floats exactly via
// &expr (ints survive evaluate) + a 2-word C28x IEEE754 reconstruction. getf() needs s/e (set below).
function getf(nm){ try { var a=Number(e.evaluate("&"+nm));
    var w=s.memory.readData(Memory.Page.DATA,a,16,2,false);
    return java.lang.Float.intBitsToFloat(((w[1]&0xFFFF)<<16)|(w[0]&0xFFFF)); } catch(err){ return NaN; } }

var ccxml=arguments[0], out=arguments[1];
var VBUS_MIN = 5.0;   // sane lower bound: bus present and the ADC is reading something real

var env=ScriptingEnvironment.instance(); env.setScriptTimeout(60000);
var server=env.getServer("DebugServer.1"); server.setConfig(ccxml);
var s=server.openSession("*","C28xx_CPU1");
s.target.connect(); s.memory.loadProgram(out);
var e=s.expression;

function fail(why){
    p("");
    p("!!!!!!!!!!!! DRV8305 gate prep FAILED !!!!!!!!!!!!");
    p("  reason: " + why);
    s.memory.writeData(Memory.Page.DATA, 0x7F0C, 0x80, 16);  // GPBCLEAR bit7 -> EN_GATE low (gate off)
    set("motorVars.flagEnableSys", "0", e);                  // stop the bg loop from driving
    p("  -> EN_GATE pulled low, flagEnableSys=0, target left HALTED. Do NOT run the lab until fixed.");
    p("=================================================");
    try { s.target.disconnect(); } catch(x){}
    server.stop(); s.terminate();
    java.lang.System.exit(1);
}

p("");
p("============ DRV8305 gate prep ============");
// 1) run init; the lab reaches "while(flagEnableSys==false)" within tens of ms.
s.target.runAsynch(); Thread.sleep(1200); s.target.halt();
var es0 = num("motorVars.flagEnableSys",e), hh = num("halHandle",e);
p("at dead-wait: flagEnableSys=" + es0 + " (expect 0)  halHandle=" + hh + " (expect nonzero)");
if(!(hh > 0))   fail("halHandle invalid -- HAL_init did not complete (not at the dead-wait).");
if(es0 !== 0)   fail("flagEnableSys != 0 -- target is not parked at the lab's enable-sys wait.");

// 2) assert EN_GATE directly (the lab skips HAL_enableDRV on DRV8305_SPI; that function is
// dead-stripped, and GpioDataRegs bitfields are not resolvable in DSS).
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
var vb = getf("motorVars.VdcBus_V");   // float: evaluate() would truncate (25.34 -> 25)
p("post-cal: flagEnableOffsetCalc=" + oc + " (expect 0)  faultUse.all=" + fu +
  " (expect 0)  VdcBus_V=" + vb + " (expect > " + VBUS_MIN + ")");
if(oc !== 0)         fail("offset cal did not complete (flagEnableOffsetCalc still 1).");
if(fu !== 0)         fail("faultUse.all=" + fu + " -- a fault is latched (e.g. over-current / bus). Power stage not healthy.");
if(!(vb > VBUS_MIN)) fail("VdcBus_V=" + vb + " below " + VBUS_MIN + " V -- bus not present or voltage sense wrong.");

// All checks passed: gate enabled, cal done, no fault, bus sane. Leave the target running.
p("");
p(">>> READY: gate enabled, offset cal done, no fault, VdcBus_V=" + vb + ".");
p("    Drive the lab's own flow from CCS (offset/gain trim, open-loop test, motor ID, ...).");
p("==========================================");
s.target.runAsynch(); s.target.disconnect(); server.stop(); s.terminate();
