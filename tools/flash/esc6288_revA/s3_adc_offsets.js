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
function set(nm,v){ try { e.evaluate(nm+"="+v); return true; } catch(err){ return false; } }
function f(x,n){ return (isNaN(x)?"nan":x.toFixed(n)); }
function rd(a){ return s.memory.readData(Memory.Page.DATA,a,16,1,false)[0]&0xFFFF; }
// emergency safe-off (used on any failure exit): force OST + de-arm (halting the CPU does not stop EPWM).
function forceSafeOff(){ try { s.memory.writeData(Memory.Page.DATA,0x409B,0x4,16);
    s.memory.writeData(Memory.Page.DATA,0x419B,0x4,16); s.memory.writeData(Memory.Page.DATA,0x429B,0x4,16);
    } catch(x){} set("motorVars.flagRunIdentAndOnLine",0); set("motorVars.flagEnableSys",0); }
// safe invariant: EPWM trip-zone one-shot (OST) set on all 3 phases = outputs forced low.
var TZFLG=[0x4093,0x4193,0x4293], OST=0x4;
function ostBit(i){ return ((rd(TZFLG[i])&OST)!=0)?1:0; }
function ostStr(){ return ostBit(0)+"/"+ostBit(1)+"/"+ostBit(2); }
function ostAllSet(){ return ostBit(0)&&ostBit(1)&&ostBit(2); }
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
    forceSafeOff(); p("  -> safe-off: OST forced, de-armed, flagEnableSys=0.");
    try { s.target.halt(); s.target.disconnect(); } catch(x){} server.stop(); s.terminate();
    java.lang.System.exit(1); }

p(""); p("======== esc6288 STAGE 3: ADC offsets / front-end (observe-only) ========");

// run startup + let the EPWM/ADC settle so SOC results are live. product.out self-enables and
// runs disarmed (gates held off by OST); the EPWM time-base triggers SOCA regardless, so the ADC
// reads are live. This stage is observe-only and never touches the power stage.
s.target.runAsynch(); Thread.sleep(1500); s.target.halt();
if(!(num("halHandle")>0)) bail("halHandle invalid -- run stage 1 first.");
if(num("motorVars.flagRunIdentAndOnLine")!=0) bail("unexpectedly ARMED -- abort before reading.");
// safe invariant: gates must be held off by the trip-zone before we read the front-end.
p("safe invariant: EPWM TZFLG.OST [1/2/3] = " + ostStr() + " (expect 1/1/1)");
if(!ostAllSet()) bail("OST not set on all phases -- power stage NOT safe-off (run stage 2 first / trip path broken).");

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
