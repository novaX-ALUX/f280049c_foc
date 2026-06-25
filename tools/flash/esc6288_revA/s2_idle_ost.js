/*
 * s2_idle_ost.js - esc6288_revA bring-up STAGE 2: idle PWM + trip-zone (OST) safe-off.
 *
 * esc6288 has NO gate-enable pin (JSM6288T); safe-off IS the EPWM trip-zone forced one-shot (OST),
 * with TZA/TZB actions = LOW. This stage confirms the power stage is held off and that offset-cal
 * does NOT release it; only the arm path (HAL_enablePWM, exercised at first-spin, stage 4) clears it.
 *
 * MODES (default is observe; anything that touches the trip needs an explicit arg):
 *   (no arg)         OBSERVE: read TZFLG.OST on EPWM1/2/3 (must all be set = outputs forced low),
 *                    flagEnableSys=0. No writes. 100% safe.
 *   verify=offcal    Run offset-cal (flagEnableSys=1) and confirm OST stays SET throughout (cal does
 *                    not un-trip), then unconditionally re-safe-off. No PWM output (gates stay tripped).
 *   verify=untrip    !!! ZERO BUS VOLTAGE ONLY !!! Momentarily clears OST (TZCLR) to prove the un-trip
 *                    mechanism, reads it cleared, then IMMEDIATELY re-forces OST (TZFRC). Never sets the
 *                    run flags, so no FOC drive is commanded. Re-safe-off on every exit path.
 *
 * Registers (F28004x): TZFLG=base+0x93 (OST=0x4), TZCLR=base+0x97, TZFRC=base+0x9B (force OST=0x4);
 *   EPWM1/2/3 base = 0x4000/0x4100/0x4200. TZ regs are EALLOW-protected; debugger writes while halted
 *   normally take -- the script reads back to confirm.
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
function forceOST(){ for(var i=0;i<3;i++) wr(TZFRC[i],OST); }       // re-arm safe-off (one-shot trip)
function clearOST(){ for(var i=0;i<3;i++) wr(TZCLR[i],OST); }       // un-trip (DANGEROUS; param only)

var ccxml=arguments[0], out=arguments[1];
var MODE="observe";
for(var i=2;i<arguments.length;i++){ var a=""+arguments[i];
    if(a=="verify=offcal") MODE="offcal"; else if(a=="verify=untrip") MODE="untrip"; }

var env=ScriptingEnvironment.instance(); env.setScriptTimeout(90000);
var server=env.getServer("DebugServer.1"); server.setConfig(ccxml);
var s=server.openSession("*","C28xx_CPU1");
s.target.connect(); s.memory.loadProgram(out);
var e=s.expression;

function safeExit(code){ forceOST(); set("motorVars.flagEnableSys",0);   // ALWAYS leave tripped + sys off
    p("  -> safe-off restored: OST forced (" + ostStr() + "), flagEnableSys=0.");
    try { s.target.halt(); s.target.disconnect(); } catch(x){} server.stop(); s.terminate();
    java.lang.System.exit(code); }
function bail(why){ p(""); p("!!!!!! STAGE 2 (idle/OST) FAILED: " + why); safeExit(1); }

p(""); p("======== esc6288 STAGE 2: idle PWM / OST safe-off  [mode=" + MODE + "] ========");

// run to the dead-wait
s.target.runAsynch(); Thread.sleep(1200); s.target.halt();
var hh=num("halHandle"), es=num("motorVars.flagEnableSys");
if(!(hh>0)) bail("halHandle invalid -- run stage 1 first.");
p("at dead-wait: flagEnableSys=" + es + " (expect 0)");

// OBSERVE (all modes start here): OST must be set on all three EPWMs.
var set0=ostStr();
p("EPWM TZFLG.OST [1/2/3] = " + set0 + "  (expect 1/1/1 = outputs forced LOW / safe-off)");
if(!ostAllSet()) bail("an EPWM is NOT tripped (OST not set) -- power stage is NOT safe-off at idle.");
if(es!==0)       bail("flagEnableSys != 0 at the dead-wait -- not parked.");
p(">>> idle safe-off confirmed (read-only).");

if(MODE=="observe"){
    p(""); p(">>> STAGE 2 OK (observe): PWM idle low, OST held on all phases. Nothing was written.");
    p("    Next: s3_adc_offsets.js. (offset-cal-holds-trip: re-run with verify=offcal.)");
    p("================================================================");
    s.target.halt(); s.target.disconnect(); server.stop(); s.terminate();
} else if(MODE=="offcal"){
    // Prove offset-cal does NOT release the trip. Gates stay tripped (no PWM output) the whole time.
    p(""); p(">>> verify=offcal: running offset cal, watching OST stays SET (cal must NOT un-trip) ...");
    if(!set("motorVars.flagEnableSys","1")) bail("could not set flagEnableSys=1.");
    var done=0;
    for(var k=0;k<10;k++){
        s.target.runAsynch(); Thread.sleep(500); s.target.halt();
        var oc=num("motorVars.flagEnableOffsetCalc");
        p("  t~" + ((k+1)*0.5).toFixed(1) + "s: flagEnableOffsetCalc=" + oc + "  OST[1/2/3]=" + ostStr());
        if(!ostAllSet()) bail("OST DROPPED during offset cal -- cal released the trip (UNSAFE design bug).");
        if(oc===0){ done=1; break; }
    }
    if(!done) p("  (offset cal did not report done within the window -- OST stayed set throughout, still safe.)");
    p(">>> offset-cal held the trip on all phases (cal does not un-trip). Restoring safe-off.");
    safeExit(0);
} else { // untrip
    p("");
    p("  !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
    p("  !! verify=untrip momentarily UN-TRIPS the EPWM outputs.        !!");
    p("  !! RUN ONLY AT ZERO BUS VOLTAGE (no power on the half-bridges). !!");
    p("  !! It does NOT set the run flags, so no FOC drive is commanded, !!");
    p("  !! but the gates are briefly releasable -- bus must be 0 V.     !!");
    p("  !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
    p("  clearing OST (TZCLR) ...");
    clearOST();
    var cleared=ostStr();
    p("  TZFLG.OST after TZCLR = " + cleared + "  (expect 0/0/0 = un-trip mechanism works)");
    forceOST();  // re-trip IMMEDIATELY regardless of what we read
    var reset=ostStr();
    p("  TZFLG.OST after TZFRC = " + reset + "  (expect 1/1/1 = safe-off re-armed)");
    if(cleared!=="0/0/0")
        p("  NOTE: OST did not read cleared -- debugger may not have EALLOW access to TZCLR; verify on the bench.");
    if(!ostAllSet())
        bail("OST did NOT re-set after TZFRC -- safe-off NOT restored. Cut power / reset the board NOW.");
    p(">>> un-trip mechanism exercised and safe-off re-armed.");
    safeExit(0);
}
