/*
 * s3_adc_offsets.js - esc6288_revA bring-up STAGE 3: ADC offsets / analog front-end. OBSERVE-ONLY.
 *
 * With the power stage held off (OST) and no bus, confirms the analog read path is alive and the
 * zero-current bias is sane BEFORE any offset cal or switching. Reads raw SOC result registers
 * (live because the EPWM time-base still triggers SOCA even while tripped) plus the firmware's
 * converted values when the ISR has populated them.
 *
 * SAFETY: pure observe, no writes, power stage untouched (OST stays forced). Run with NO bus first;
 * a known bench voltage on the bus is optional (to sanity-check the Udc divider).
 *
 * esc6288 ADC map (SOC result regs): IA=ADCB SOC0=0x0B20, IB=ADCA SOC0=0x0B00, IC=ADCC SOC0=0x0B40,
 *   Udc=ADCA SOC1=0x0B01, NTC=ADCC SOC2=0x0B42. Bidirectional INA181 -> zero current ~2048 (1.65 V).
 *
 * Usage: dss.sh tools/flash/esc6288_revA/s3_adc_offsets.js <ccxml> <product.out>
 */
importPackage(Packages.com.ti.debug.engine.scripting);
importPackage(Packages.com.ti.ccstudio.scripting.environment);
importPackage(Packages.java.lang);
function p(s){ System.out.println(s); }
function num(nm){ try { return Number(e.evaluate(nm)); } catch(err){ return NaN; } }
function f(x,n){ return (isNaN(x)?"nan":x.toFixed(n)); }
function rd(a){ return s.memory.readData(Memory.Page.DATA,a,16,1,false)[0]&0xFFFF; }
function getf(nm){ try { var a=Number(e.evaluate("&"+nm));
    var w=s.memory.readData(Memory.Page.DATA,a,16,2,false);
    return java.lang.Float.intBitsToFloat(((w[1]&0xFFFF)<<16)|(w[0]&0xFFFF)); } catch(err){ return NaN; } }

// NTC count -> degC, mirroring src/common/ntc.c + board.h (NCP18XH103, high-side, R14=10k, 4096 FS).
function ntcC(cnt){
    if(cnt<2 || cnt>4094) return NaN;            // open/short guard
    var ratio=cnt/4096.0, r=10000.0*(1.0-ratio)/ratio;
    if(r<=0) return NaN;
    var t=1.0/(1.0/298.15 + Math.log(r/10000.0)/3380.0) - 273.15;
    return t;
}

var ccxml=arguments[0], out=arguments[1];
var env=ScriptingEnvironment.instance(); env.setScriptTimeout(60000);
var server=env.getServer("DebugServer.1"); server.setConfig(ccxml);
var s=server.openSession("*","C28xx_CPU1");
s.target.connect(); s.memory.loadProgram(out);
var e=s.expression;

function bail(why){ p(""); p("!!!!!! STAGE 3 (ADC) FAILED: " + why);
    try { s.target.halt(); s.target.disconnect(); } catch(x){} server.stop(); s.terminate();
    java.lang.System.exit(1); }

p(""); p("======== esc6288 STAGE 3: ADC offsets / front-end (observe-only) ========");

// run to dead-wait + let the EPWM/ADC settle so SOC results are live.
s.target.runAsynch(); Thread.sleep(1200); s.target.halt();
if(!(num("halHandle")>0)) bail("halHandle invalid -- run stage 1 first.");

// raw zero-current counts (mid-rail ~2048). EPWM SOCA triggers even while tripped.
var ia=rd(0x0B20), ib=rd(0x0B00), ic=rd(0x0B40), udc=rd(0x0B01), ntc=rd(0x0B42);
p("");
p("--- raw SOC counts (12-bit) ---");
p("  I_A: A=" + ia + "  B=" + ib + "  C=" + ic + "   (expect ~2048 = 1.65 V mid-rail bias)");
p("  Udc_raw=" + udc + "   NTC_raw=" + ntc);
var bad=0;
function chkmid(nm,c){ if(c<1500||c>2600){ p("  WARNING: " + nm + " count " + c + " far from mid-scale 2048 -- check INA181 bias / ADC mux."); bad++; } }
chkmid("IA",ia); chkmid("IB",ib); chkmid("IC",ic);

// NTC -> degC (room temp ~2048 since R14 and the NCP18XH103 are both 10k).
var tC=ntcC(ntc);
p("  NTC: raw=" + ntc + " -> " + (isNaN(tC)?"OPEN/SHORT (near rail)":f(tC,1)+" C") + "   (room temp expect ~2048 / ~25 C)");
if(isNaN(tC)) p("  WARNING: NTC reads open/short -- check the thermistor / R14 divider before trusting over-temp.");
else if(tC<0||tC>60) p("  WARNING: NTC " + f(tC,1) + " C is outside a typical bench ambient -- verify divider/curve.");

// firmware converted values (populated once the ISR runs; report if available).
p("");
p("--- firmware converted (if ISR populated) ---");
p("  VdcBus_V = " + f(getf("motorVars.VdcBus_V"),3) + "   (no bus: ~0; bench bus: ~applied volts)");
p("  adcData.I_A = " + f(getf("adcData.I_A.value[0]"),3) + " / " + f(getf("adcData.I_A.value[1]"),3) +
  " / " + f(getf("adcData.I_A.value[2]"),3) + " A  (zero current: ~0 after offset)");
p("  offsets_I_A = " + f(getf("motorVars.offsets_I_A.value[0]"),3) + " / " +
  f(getf("motorVars.offsets_I_A.value[1]"),3) + " / " + f(getf("motorVars.offsets_I_A.value[2]"),3) +
  "   (populated only after is02 offset cal)");

p("");
if(bad>0) p(">>> STAGE 3: completed with " + bad + " warning(s) -- judge against the bench before is02.");
else      p(">>> STAGE 3 OK: zero-current bias ~mid-scale, NTC sane. Front-end read path looks live.");
p("    Next: s5_protection.js (trip-zone / CMPSS / OC latch checks).");
p("================================================================");
s.target.halt(); s.target.disconnect(); server.stop(); s.terminate();
