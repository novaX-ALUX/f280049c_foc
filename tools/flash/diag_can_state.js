/* CAN bring-up diagnostic: read the can_bridge FIFO/TX state + CANA error counters to tell
 * "target TX with no ACK" (bus-off / TEC high -> physical/bitrate/termination) apart from
 * "target idle" (nothing queued). Attaches, samples twice ~1s apart, leaves target running.
 * Usage: dss.sh tools/flash/diag_can_state.js <ccxml> <product.out>  */
importPackage(Packages.com.ti.debug.engine.scripting);
importPackage(Packages.com.ti.ccstudio.scripting.environment);
importPackage(Packages.java.lang);
function p(s){ System.out.println(s); }
function num(x){ try { return Number(e.evaluate(x)); } catch(err){ return NaN; } }
function reg32(addr){ try { var w=s.memory.readData(Memory.Page.DATA,addr,32,1,false);
    return Number(w[0])>>>0; } catch(err){ return NaN; } }
function hx(v){ return isNaN(v)?"n/a":("0x"+(v>>>0).toString(16)); }

var ccxml=arguments[0], out=arguments[1];
var env=ScriptingEnvironment.instance(); env.setScriptTimeout(60000);
var server=env.getServer("DebugServer.1"); server.setConfig(ccxml);
var s=server.openSession("*","C28xx_CPU1"); s.target.connect(); s.memory.loadProgram(out);
var e=s.expression;
var CANA=0x00048000;

s.target.runAsynch(); Thread.sleep(1500); s.target.halt();
function snap(tag){
    p("---- "+tag+" ----");
    p("  g_now_ms              : "+num("g_now_ms"));
    p("  g_dn.node_id          : "+num("g_dn.node_id"));
    p("  g_dn.tid_node_status  : "+num("g_dn.tid_node_status")+"   (advances => dronecan_tick is emitting)");
    p("  s_ints_enabled        : "+num("s_ints_enabled"));
    p("  s_tx_in_flight        : "+num("s_tx_in_flight")+"   (stuck 1 => TX never completes/ACKs)");
    p("  s_tx head/tail/dropped: "+num("s_tx.head")+"/"+num("s_tx.tail")+"/"+num("s_tx.dropped"));
    p("  s_rx head/tail/dropped: "+num("s_rx.head")+"/"+num("s_rx.tail")+"/"+num("s_rx.dropped"));
    var es=reg32(CANA+0x4), errc=reg32(CANA+0x8), ctl=reg32(CANA+0x0);
    p("  CAN_CTL               : "+hx(ctl));
    if(!isNaN(es)){
        p("  CAN_ES                : "+hx(es)
          +"  [LEC="+(es&0x7)+" TXOK="+((es>>3)&1)+" RXOK="+((es>>4)&1)
          +" EPASS="+((es>>5)&1)+" EWARN="+((es>>6)&1)+" BOFF="+((es>>7)&1)+"]");
    } else { p("  CAN_ES                : n/a (reg read blocked)"); }
    if(!isNaN(errc)){ p("  CAN_ERRC              : TEC="+(errc&0xFF)+"  REC="+((errc>>8)&0x7F)
          +"   (TEC>127 or BOFF => no ACK on the bus)"); }
}
snap("sample 1");
s.target.runAsynch(); Thread.sleep(1200); s.target.halt();
snap("sample 2 (~1.2s later)");
p("");
p(">>> read: tid_node_status advancing + TEC climbing/BOFF=1  => target IS transmitting, bus not ACKing");
p(">>>       (check 1 Mbit bitrate match + 120ohm termination + CANH/CANL wiring).");
p(">>>       LEC 3=ack-error confirms 'I sent, nobody acknowledged'.");
s.target.runAsynch(); s.target.disconnect(); server.stop(); s.terminate();
