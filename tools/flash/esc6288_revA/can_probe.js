/*
 * can_probe.js - esc6288_revA CAN liveness probe (READ-ONLY).
 * Reads the file-static FIFOs s_rx/s_tx in can_bridge.c by ADDRESS (symbol.getAddress, since the DSS
 * expression evaluator can't see file-statics), plus s_tx_in_flight and g_dn, across two ~1.5 s samples.
 * dronecan_fifo_t layout (C28x 16-bit words): head@+0 (u16), tail@+1 (u16), dropped@+2..+3 (u32).
 * Interpretation:
 *   s_tx.head advancing  -> firmware is ENQUEUEING TX frames (DroneCAN stack producing output)
 *   s_tx.tail advancing   -> TX frames DRAIN onto the bus (transmits complete => a partner is ACKing)
 *   s_tx.head grows but tail stuck / in_flight stuck true -> TX NOT completing (no ACK / bus-off / wiring/term/baud)
 *   s_rx.head advancing   -> frames RECEIVED
 * SAFETY: pure read; never writes; gates stay OST-tripped. (No DAC/PWM/arm touched.)
 */
importPackage(Packages.com.ti.debug.engine.scripting);
importPackage(Packages.com.ti.ccstudio.scripting.environment);
importPackage(Packages.java.lang);
function p(s){ System.out.println(s); }
function num(nm){ try { return Number(e.evaluate(nm)); } catch(err){ return NaN; } }
function rdw(a){ return s.memory.readData(Memory.Page.DATA,a,16,1,false)[0]&0xFFFF; }
function rd32(a){ return (rdw(a) | (rdw(a+1)<<16))>>>0; }
// file-static FIFO addresses passed in (DSS can't resolve local symbols; the wrapper reads them via nm)
var ccxml=arguments[0], out=arguments[1];
var aRx=parseInt(arguments[2],16), aTx=parseInt(arguments[3],16), aInf=parseInt(arguments[4],16);
var env=ScriptingEnvironment.instance(); env.setScriptTimeout(120000);
var server=env.getServer("DebugServer.1"); server.setConfig(ccxml);
var s=server.openSession("*","C28xx_CPU1");
s.target.connect(); s.memory.loadProgram(out);
var e=s.expression;
function settle(ms){ s.target.runAsynch(); Thread.sleep(ms); s.target.halt(); }
function done(c){ try { s.target.halt(); s.target.disconnect(); } catch(x){} server.stop(); s.terminate(); java.lang.System.exit(c); }

p(""); p("======== esc6288 CAN probe (read-only) ========");
settle(1500);
if(!(num("halHandle")>0)){ p("halHandle invalid -- run stage 1 first."); done(1); }

p("symbols (from nm): &s_rx=0x"+aRx.toString(16)+"  &s_tx=0x"+aTx.toString(16)+"  &s_tx_in_flight=0x"+aInf.toString(16));
if(isNaN(aRx)||isNaN(aTx)){ p("  bad FIFO addresses passed in."); done(1); }

function fifo(a){ return {head:rdw(a), tail:rdw(a+1), dropped:rd32(a+2)}; }
function sample(tag){
    var rx=fifo(aRx), tx=fifo(aTx), inf=isNaN(aInf)?-1:rdw(aInf);
    p("--- "+tag+" ---");
    p("  s_rx: head="+rx.head+" tail="+rx.tail+" dropped="+rx.dropped+"   (head!=tail => unconsumed RX frames)");
    p("  s_tx: head="+tx.head+" tail="+tx.tail+" dropped="+tx.dropped+"   in_flight="+inf);
    p("  g_dn.node_id="+num("g_dn.node_id")+"  armed="+num("g_dn.armed"));
    return {rx:rx, tx:tx, inf:inf};
}
var s1=sample("sample 1");
settle(1500);
var s2=sample("sample 2 (~1.5 s later)");

p("");
p(">>> movement over ~1.5 s:");
p("    TX produced (s_tx.head):  "+s1.tx.head+" -> "+s2.tx.head+(s2.tx.head!=s1.tx.head?"  (firmware IS enqueuing TX)":"  (no new TX enqueued)"));
p("    TX drained  (s_tx.tail):  "+s1.tx.tail+" -> "+s2.tx.tail+(s2.tx.tail!=s1.tx.tail?"  (TX completing on the bus => partner ACKing)":"  (TX NOT draining)"));
p("    RX received (s_rx.head):  "+s1.rx.head+" -> "+s2.rx.head+(s2.rx.head!=s1.rx.head?"  (frames RECEIVED)":"  (no RX)"));
var txStuck = (s2.tx.head!=s2.tx.tail) && (s2.tx.tail==s1.tx.tail) && (s2.tx.head!=s1.tx.head || s1.tx.head!=s1.tx.tail);
if(txStuck) p("    >>> TX enqueued but NOT draining -> CAN not transmitting/acked (check term 120R, baud 1 Mbit, TX/RX pins, transceiver pwr).");
p("================================================================");
done(0);
