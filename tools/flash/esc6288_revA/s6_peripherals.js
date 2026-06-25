/*
 * s6_peripherals.js - esc6288_revA bring-up STAGE 6: CAN / encoder / RC-PWM / RGB. READ-ONLY.
 *
 * Confirms the aux interfaces are alive without touching the power stage. Takes two samples ~1.5 s
 * apart so operator action shows up: send DroneCAN frames (s_rx advances), rotate the magnet
 * (g_enc.position_rev changes), feed a 1-2 ms RC pulse on GPIO33 (eCAP CAP2 tracks the high-time).
 *
 * SAFETY: pure observe -- no PWM, no un-trip, no motion. The power stage stays held off (OST).
 *
 * Sources: can_bridge s_rx/s_tx + g_dn (symbol reads, board-agnostic); MT6701 processed state
 *   g_enc (src/encoder/mt6701); RC-PWM eCAP1 CAP2 = 0x5206 (high-time in SYSCLK counts, 100/us).
 *   RGB (WS2812 on GPIO12) is bit-banged with tight timing and CANNOT be driven from a halted
 *   debugger -- confirm it visually while the product main runs (see note below).
 *
 * Usage: dss.sh tools/flash/esc6288_revA/s6_peripherals.js <ccxml> <product.out>
 */
importPackage(Packages.com.ti.debug.engine.scripting);
importPackage(Packages.com.ti.ccstudio.scripting.environment);
importPackage(Packages.java.lang);
function p(s){ System.out.println(s); }
function num(nm){ try { return Number(e.evaluate(nm)); } catch(err){ return NaN; } }
function f(x,n){ return (isNaN(x)?"nan":x.toFixed(n)); }
function rd32(a){ var w=s.memory.readData(Memory.Page.DATA,a,16,2,false); return ((w[1]&0xFFFF)<<16)|(w[0]&0xFFFF); }
function getf(nm){ try { var a=Number(e.evaluate("&"+nm));
    var w=s.memory.readData(Memory.Page.DATA,a,16,2,false);
    return java.lang.Float.intBitsToFloat(((w[1]&0xFFFF)<<16)|(w[0]&0xFFFF)); } catch(err){ return NaN; } }

var ECAP1_CAP2=0x5206;   // eCAP1 CAP2: RC-PWM high-time in SYSCLK counts (100 MHz -> 100 counts/us)
function rcUs(){ return rd32(ECAP1_CAP2)/100.0; }
function rcThrottle(us){ var t=(us-1000.0)/1000.0; return (t<0)?0:((t>1)?1:t); }

var ccxml=arguments[0], out=arguments[1];
var env=ScriptingEnvironment.instance(); env.setScriptTimeout(60000);
var server=env.getServer("DebugServer.1"); server.setConfig(ccxml);
var s=server.openSession("*","C28xx_CPU1");
s.target.connect(); s.memory.loadProgram(out);
var e=s.expression;

function bail(why){ p(""); p("!!!!!! STAGE 6 (peripherals) FAILED: " + why);
    try { s.target.halt(); s.target.disconnect(); } catch(x){} server.stop(); s.terminate();
    java.lang.System.exit(1); }

function snap(tag){
    p("--- " + tag + " ---");
    // CAN (symbol reads -- board-agnostic)
    p("  CAN: s_rx h/t=" + num("s_rx.head") + "/" + num("s_rx.tail") + " drop=" + num("s_rx.dropped") +
      " | s_tx h/t=" + num("s_tx.head") + "/" + num("s_tx.tail") + " drop=" + num("s_tx.dropped") +
      " in_flight=" + num("s_tx_in_flight"));
    p("       g_dn.node_id=" + num("g_dn.node_id") + "  armed=" + num("g_dn.armed"));
    // encoder (MT6701 processed)
    p("  ENC: pos_rev=" + f(getf("g_enc.position_rev"),4) + " vel_revps=" + f(getf("g_enc.vel_revps"),3) +
      " valid=" + num("g_enc.valid") + " stale=" + num("g_enc.stale") + " glitch=" + num("g_enc.glitch_count"));
    // RC-PWM (eCAP capture)
    var us=rcUs();
    p("  RC : high-time=" + f(us,1) + " us -> throttle=" + f(rcThrottle(us),3) +
      ((us<900||us>2100) ? "  (out of 1-2 ms band: no signal / glitch?)" : ""));
}

p(""); p("======== esc6288 STAGE 6: CAN / encoder / RC-PWM / RGB (read-only) ========");
if(num("halHandle")>0){} // load already done; run to populate live state
s.target.runAsynch(); Thread.sleep(1500); s.target.halt();
snap("sample 1");
p("  (now: send a DroneCAN frame / rotate the magnet / feed an RC pulse to see the values move)");
s.target.runAsynch(); Thread.sleep(1500); s.target.halt();
snap("sample 2 (~1.5 s later)");

p("");
p("  RGB (WS2812 GPIO12): cannot be driven from the halted debugger (bit-bang timing). Watch the LED");
p("  while the product main runs and confirm the status colors change; scope GPIO12 if tuning WS_*_LOOPS.");
p("");
p(">>> STAGE 6: read CAN/encoder/RC-PWM state across two samples. Judge movement vs your operator action.");
p("    Pre-first-spin bring-up (stages 1,2,3,5,6) complete -- stage 4 (first spin) is a separate step.");
p("================================================================");
s.target.halt(); s.target.disconnect(); server.stop(); s.terminate();
