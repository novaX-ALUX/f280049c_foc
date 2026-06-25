/*
 * s2_idle_ost.js - esc6288_revA bring-up STAGE 2: idle PWM + trip-zone (OST) safe-off.
 *
 * Target: the product image (product.out). Unlike the SDK is01 lab there is NO flagEnableSys==0
 * dead-wait -- product_main self-sets flagEnableSys=true and runs offset cal automatically. The
 * power stage is still held OFF the whole time by the EPWM trip-zone one-shot (OST, TZA/TZB=LOW);
 * the gates only switch when the arm path (HAL_enablePWM) fires, which needs flagRunIdentAndOnLine
 * AND offset cal done AND a run command. So the safe invariant on product.out is:
 *     OST set on EPWM1/2/3  AND  flagRunIdentAndOnLine == 0 (not armed).
 * (flagEnableSys is 1 by design and is NOT a safety indicator.)
 *
 * This stage confirms that invariant holds across startup INCLUDING the auto offset cal (cal must
 * not release the trip), and optionally exercises the un-trip mechanism.
 *
 * MODES (default observe writes NOTHING; anything that touches the trip needs an explicit arg):
 *   (no arg)       OBSERVE: sample OST through startup+cal; confirm it stays set and never armed.
 *   verify=offcal  Same sampling, reported verbosely as the "cal does not un-trip" check.
 *   verify=untrip  !!! ZERO BUS VOLTAGE ONLY !!! Momentarily clears OST (TZCLR) to prove the un-trip
 *                  mechanism, then IMMEDIATELY re-forces OST (TZFRC). Never sets run flags.
 *
 * Any explicit-arg path exits via safeExit(): force OST + flagRunIdentAndOnLine=0 + flagEnableSys=0.
 * Registers: TZFLG=base+0x93 (OST=0x4), TZCLR=base+0x97, TZFRC=base+0x9B; EPWM1/2/3=0x4000/0x4100/0x4200.
 *
 * Usage: dss.sh tools/flash/esc6288_revA/s2_idle_ost.js <ccxml> <product.out> [verify=offcal|verify=untrip]
 */
importPackage(Packages.com.ti.debug.engine.scripting);
importPackage(Packages.com.ti.ccstudio.scripting.environment);
importPackage(Packages.java.lang);
function p(s){ System.out.println(s); }
function num(nm){ try { return Number(e.evaluate(nm)); } catch(err){ return NaN; } }
function set(nm,v){ try { e.evaluate(nm+"="+v); return true; } catch(err){ return false; } }
function rd(a){ return s.memory.readData(Memory.Page.DATA,a,16,1,false)[0]&0xFFFF; }
function wr(a,v){ try { s.memory.writeData(Memory.Page.DATA,a,v,16); } catch(x){} }

var TZFLG=[0x4093,0x4193,0x4293], TZCLR=[0x4097,0x4197,0x4297], TZFRC=[0x409B,0x419B,0x429B];
var OST=0x4;
function ostBit(i){ return ((rd(TZFLG[i])&OST)!=0)?1:0; }
function ostStr(){ return ostBit(0)+"/"+ostBit(1)+"/"+ostBit(2); }
function ostAllSet(){ return ostBit(0)&&ostBit(1)&&ostBit(2); }
function armed(){ return num("motorVars.flagRunIdentAndOnLine")!=0; }
function forceOST(){ for(var i=0;i<3;i++) wr(TZFRC[i],OST); }       // re-arm safe-off (one-shot trip)
function clearOST(){ for(var i=0;i<3;i++) wr(TZCLR[i],OST); }       // un-trip (DANGEROUS; param only)

var ccxml=arguments[0], out=arguments[1];
var MODE="observe";
for(var i=2;i<arguments.length;i++){ var a=""+arguments[i];
    if(a=="verify=offcal") MODE="offcal"; else if(a=="verify=untrip") MODE="untrip"; }

var env=ScriptingEnvironment.instance(); env.setScriptTimeout(120000);
var server=env.getServer("DebugServer.1"); server.setConfig(ccxml);
var s=server.openSession("*","C28xx_CPU1");
s.target.connect(); s.memory.loadProgram(out);
var e=s.expression;

function safeExit(code){ forceOST(); set("motorVars.flagRunIdentAndOnLine",0); set("motorVars.flagEnableSys",0);
    p("  -> safe-off: OST forced (" + ostStr() + "), flagRunIdentAndOnLine=0, flagEnableSys=0.");
    try { s.target.halt(); s.target.disconnect(); } catch(x){} server.stop(); s.terminate();
    java.lang.System.exit(code); }
function plainExit(){ try { s.target.halt(); s.target.disconnect(); } catch(x){} server.stop(); s.terminate(); }
function bail(why){ p(""); p("!!!!!! STAGE 2 (idle/OST) FAILED: " + why); safeExit(1); }

p(""); p("======== esc6288 STAGE 2: idle PWM / OST safe-off  [mode=" + MODE + "] ========");

// Sample OST through startup + the automatic offset cal. The gates must stay tripped the whole
// time (cal must NOT un-trip) and the product must never arm itself with no run command.
var calSeen=0, calDone=0;
for(var k=0;k<10;k++){
    s.target.runAsynch(); Thread.sleep(300); s.target.halt();
    if(k===0 && !(num("halHandle")>0)) bail("halHandle invalid -- run stage 1 first.");
    var oc=num("motorVars.flagEnableOffsetCalc");
    if(oc!==0) calSeen=1;
    if(calSeen && oc===0) calDone=1;
    if(!ostAllSet()) bail("OST DROPPED during startup/offset-cal -- gates released the trip (UNSAFE).");
    if(armed())      bail("flagRunIdentAndOnLine set with no run command -- unexpected self-arm. ABORT.");
    if(k%3===0 || calDone)
        p("  t~" + ((k+1)*0.3).toFixed(1) + "s: OST[1/2/3]=" + ostStr() + " offsetCalc=" + oc +
          " armed=" + (armed()?1:0));
    if(calDone && k>=3) break;
}
p("startup invariant: OST=" + ostStr() + " (expect 1/1/1)  armed=" + (armed()?1:0) + " (expect 0)  " +
  "offsetCal " + (calDone?"completed":(calSeen?"in progress":"not observed")) + "; flagEnableSys=" +
  num("motorVars.flagEnableSys") + " (1 by design, not a safety flag)");
if(!ostAllSet()) bail("OST not set after startup.");
if(armed())      bail("armed after startup.");
p(">>> idle safe-off confirmed: gates held off by OST through startup+cal, not armed.");

if(MODE=="observe"){
    p(""); p(">>> STAGE 2 OK (observe): nothing written. Next: s3_adc_offsets.js.");
    p("    (cal-holds-trip detail: verify=offcal;  un-trip mechanism: verify=untrip.)");
    p("================================================================");
    plainExit();
} else if(MODE=="offcal"){
    p(""); p(">>> verify=offcal: the sampling above ran across the automatic offset cal.");
    if(calSeen) p("    offset cal was observed in progress and OST never dropped -> cal does NOT un-trip.");
    else        p("    offset cal completed before the first sample; OST held on every sample (still safe).");
    safeExit(0);
} else { // untrip
    p("");
    p("  !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
    p("  !! verify=untrip momentarily UN-TRIPS the EPWM outputs.        !!");
    p("  !! RUN ONLY AT ZERO BUS VOLTAGE (no power on the half-bridges). !!");
    p("  !! Run flags are NOT set, so no FOC drive is commanded.        !!");
    p("  !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
    p("  clearing OST (TZCLR) ...");
    clearOST();
    var cleared=ostStr();
    p("  TZFLG.OST after TZCLR = " + cleared + "  (expect 0/0/0 = un-trip mechanism works)");
    forceOST();   // re-trip IMMEDIATELY regardless of what we read
    p("  TZFLG.OST after TZFRC = " + ostStr() + "  (expect 1/1/1 = safe-off re-armed)");
    if(cleared!=="0/0/0")
        p("  NOTE: OST did not read cleared -- debugger may lack EALLOW access to TZCLR; verify on the bench.");
    if(!ostAllSet())
        bail("OST did NOT re-set after TZFRC -- safe-off NOT restored. Cut power / reset the board NOW.");
    p(">>> un-trip mechanism exercised and safe-off re-armed.");
    safeExit(0);
}
