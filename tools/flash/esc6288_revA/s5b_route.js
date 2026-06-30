/*
 * s5b_route.js - esc6288_revA STAGE 5b: CMPSS comparator + input-mux routing test (SAFE, no injection).
 *
 * Proves the hardware OC/OV comparators actually sense their analog inputs and assert, WITHOUT applying
 * real over-current or over-voltage. Method: temporarily LOWER each high-side DAC threshold BELOW the
 * resting signal so the resting level itself trips the high comparator; read COMPSTS; then RESTORE the
 * original threshold and clear the latch.
 *
 *   CMPSS3 high (phase-C current, ADCINC2): resting ~2048 (1.65 V). Lower DACH < 2048 -> COMPHSTS=1.
 *   CMPSS5 high (DC-bus, ADCINA6):          resting ~bus count. Lower DACH < bus -> COMPHSTS=1.
 *
 * SAFETY: lowering an OC/OV threshold only makes protection MORE sensitive (safe-off direction). Gates
 * stay OST-tripped the whole time (we never clear OST, never arm, never drive PWM). Original DAC values
 * are restored and latches cleared; all scripted exit paths force OST + disarm (a connect/load failure
 * or a thrown read error exits without that cleanup, but only ever leaves a threshold LOWERED = safe-off
 * direction, and the next DSS run re-inits HAL). (Each DSS run reloads the RAM
 * image and re-inits HAL anyway, so changes are transient regardless.)
 *
 * Usage: dss.sh s5b_route.js <ccxml> <product.out>
 */
importPackage(Packages.com.ti.debug.engine.scripting);
importPackage(Packages.com.ti.ccstudio.scripting.environment);
importPackage(Packages.java.lang);
function p(s){ System.out.println(s); }
function num(nm){ try { return Number(e.evaluate(nm)); } catch(err){ return NaN; } }
function set(nm,v){ try { e.evaluate(nm+"="+v); return true; } catch(err){ return false; } }
function rd(a){ return s.memory.readData(Memory.Page.DATA,a,16,1,false)[0]&0xFFFF; }
function wr(a,v){ try { s.memory.writeData(Memory.Page.DATA,a,v,16); } catch(x){} }

// trip-zone (safe-off)
var TZFLG=[0x4093,0x4193,0x4293], TZFRC=[0x409B,0x419B,0x429B], OST=0x4;
function ostBit(i){ return ((rd(TZFLG[i])&OST)!=0)?1:0; }
function ostStr(){ return ostBit(0)+"/"+ostBit(1)+"/"+ostBit(2); }
function ostAllSet(){ return ostBit(0)&&ostBit(1)&&ostBit(2); }
function forceOST(){ for(var i=0;i<3;i++) wr(TZFRC[i],OST); }

// CMPSS3 (phase-C OC) and CMPSS5 (bus OV): base+2=COMPSTS, +3=COMPSTSCLR, +6=DACHVALS(shadow), +7=DACHVALA
var C3={sts:0x5CC2, clr:0x5CC3, dacs:0x5CC6, daca:0x5CC7, sig:0x0B40, name:"CMPSS3 phase-C OC (ADCINC2)"};
var C5={sts:0x5D02, clr:0x5D03, dacs:0x5D06, daca:0x5D07, sig:0x0B01, name:"CMPSS5 bus OV (ADCINA6)"};
var HLIVE=0x1, HLATCH=0x2, LLATCH=0x200, HLATCHCLR=0x2, LLATCHCLR=0x200;
function stsStr(a){ var v=rd(a); return "Hlive="+((v&HLIVE)?1:0)+" Hlatch="+((v&HLATCH)?1:0)+
    " Llatch="+((v&LLATCH)?1:0)+" (0x"+v.toString(16)+")"; }

var ccxml=arguments[0], out=arguments[1];
var env=ScriptingEnvironment.instance(); env.setScriptTimeout(120000);
var server=env.getServer("DebugServer.1"); server.setConfig(ccxml);
var s=server.openSession("*","C28xx_CPU1");
s.target.connect(); s.memory.loadProgram(out);
var e=s.expression;
function settle(ms){ s.target.runAsynch(); Thread.sleep(ms); s.target.halt(); }
function safeExit(code){ forceOST(); set("motorVars.flagRunIdentAndOnLine",0); set("motorVars.flagEnableSys",0);
    p("  -> safe-off: OST forced ("+ostStr()+"), disarmed.");
    try { s.target.halt(); s.target.disconnect(); } catch(x){} server.stop(); s.terminate(); java.lang.System.exit(code); }
function bail(why){ p(""); p("!!!!!! STAGE 5b FAILED: "+why); safeExit(1); }

p(""); p("======== esc6288 STAGE 5b: CMPSS comparator/route test (no injection) ========");
settle(1500);
if(!(num("halHandle")>0)) bail("halHandle invalid -- run stage 1 first.");
if(num("motorVars.flagRunIdentAndOnLine")!=0) bail("ARMED -- abort.");
if(!ostAllSet()) bail("OST not set at baseline -- not safe-off (run stage 2 first).");
p("baseline: OST="+ostStr()+" (gates safe-off)  "+C3.name+" "+stsStr(C3.sts)+"  / bus-OV "+stsStr(C5.sts));

// returns true if the high comparator asserted (live or latched) after lowering its threshold
function testCmpss(c){
    var sig=rd(c.sig);
    var dacBase=rd(c.dacs);                              // saved shadow (the real OC/OV threshold)
    p("");
    p("--- "+c.name+" ---");
    p("  resting signal count = "+sig+" ;  baseline DACH(shadow)="+dacBase+"  COMPSTS "+stsStr(c.sts));
    if(sig<40){ p("  SKIP: resting signal ~0 (no bus voltage?) -- cannot trip by lowering threshold."); return null; }
    var newDac=Math.max(20, Math.floor(sig/2));          // safely below the resting signal
    p("  lowering DACH to "+newDac+" (< signal) so the resting level itself trips the high comparator ...");
    wr(c.dacs, newDac);
    settle(400);                                         // sysclk loads shadow->active; filter settles
    var after=rd(c.sts);
    var tripped=((after&HLIVE)||(after&HLATCH))!=0;
    p("  after lowering: COMPSTS "+stsStr(c.sts)+"  -> high comparator "+(tripped?"TRIPPED":"did NOT trip"));
    // restore + clear latch
    wr(c.dacs, dacBase);
    wr(c.clr, HLATCHCLR|LLATCHCLR);
    settle(200);
    p("  restored DACH="+rd(c.dacs)+" ; cleared latch -> COMPSTS "+stsStr(c.sts)+"  (OST still "+ostStr()+")");
    return tripped;
}

var r5=testCmpss(C5);   // bus OV first (clearly single-ended high)
var r3=testCmpss(C3);   // phase-C OC

p("");
function verdict(name,r){ return name+": "+(r===null?"SKIPPED":(r?"PASS (comparator+input+DAC OK)":"FAIL (did not assert)")); }
p(">>> ROUTE TEST RESULT:");
p("    "+verdict("CMPSS5 bus-OV ", r5));
p("    "+verdict("CMPSS3 phase-C", r3));
p("    (Proves each comparator senses its analog input + DAC threshold. The CMPSS->X-BAR->OST");
p("     end-to-end trip still needs real OC/OV injection at first-spin, or a zero-bus un-trip test.)");
if(!ostAllSet()) bail("OST somehow dropped during test -- investigate before proceeding.");
p("================================================================");
safeExit(0);
