/* Read can_bridge TX/RX FIFO + flags by ADDRESS (symbols are local/stripped from eval).
 * Addresses from ofd2000 -g on esc0_node25/product.out:
 *   s_tx_in_flight 0xcf80, s_ints_enabled 0xcf81, s_rx 0xcf82, s_tx 0xd046.
 * dronecan_fifo_t = { uint16 head; uint16 tail; uint32 dropped; frame buf[16]; }
 * Two samples ~1.2s apart: tail moving + dropped==0 => TX draining (frames ACKed/leaving);
 * tail frozen + dropped climbing => TX stuck (no ACK on the bus -> wiring/adapter PHY).
 * Usage: dss.sh tools/flash/diag_tx_state.js <ccxml> <product.out>  */
importPackage(Packages.com.ti.debug.engine.scripting);
importPackage(Packages.com.ti.ccstudio.scripting.environment);
importPackage(Packages.java.lang);
function p(s){ System.out.println(s); }
function w16(a){ return s.memory.readData(Memory.Page.DATA,a,16,1,false)[0]&0xFFFF; }
function w32(a){ var w=s.memory.readData(Memory.Page.DATA,a,16,2,false);
    return ((w[1]&0xFFFF)<<16)|(w[0]&0xFFFF); }
var ccxml=arguments[0], out=arguments[1];
var env=ScriptingEnvironment.instance(); env.setScriptTimeout(60000);
var server=env.getServer("DebugServer.1"); server.setConfig(ccxml);
var s=server.openSession("*","C28xx_CPU1"); s.target.connect(); s.memory.loadProgram(out);
var TXIF=0xcf80, INTS=0xcf81, RX=0xcf82, TX=0xd046;
function snap(tag){
    p("---- "+tag+" ----");
    p("  s_ints_enabled   : "+w16(INTS)+"        (expect 1)");
    p("  s_tx_in_flight   : "+w16(TXIF)+"        (stuck 1 => TX not completing)");
    p("  s_tx head/tail   : "+w16(TX)+"/"+w16(TX+1)+"   dropped="+w32(TX+2)
      +"   (head!=tail => queued; dropped climbing => can't send)");
    p("  s_rx head/tail   : "+w16(RX)+"/"+w16(RX+1)+"   dropped="+w32(RX+2)
      +"   (head!=tail => RX frames arrived!)");
}
s.target.runAsynch(); Thread.sleep(1500); s.target.halt(); snap("sample 1");
s.target.runAsynch(); Thread.sleep(1200); s.target.halt(); snap("sample 2 (~1.2s)");
p("");
p(">>> ints=1 + tail FROZEN + dropped CLIMBING => target drives bus, gets no ACK (PHY/wiring/adapter).");
p(">>> ints=1 + tail MOVING + dropped 0       => TX completes (someone ACKs) -> adapter SHOULD see it.");
p(">>> s_rx head!=tail                          => we are RECEIVING (adapter TX reaches us).");
s.target.runAsynch(); s.target.disconnect(); server.stop(); s.terminate();
