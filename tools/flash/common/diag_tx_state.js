/* Read can_bridge TX/RX FIFO + flags by ADDRESS (s_* are file-static, often stripped from eval).
 * dronecan_fifo_t = { uint16 head; uint16 tail; uint32 dropped; frame buf[16]; }
 * Two samples ~1.2s apart: tail moving + dropped==0 => TX draining (frames ACKed/leaving);
 * tail frozen + dropped climbing => TX stuck (no ACK on the bus -> wiring/adapter PHY).
 *
 * Addresses are PER-BUILD -- the defaults below are launchxl_drv8305evm (esc0_node25/product.out).
 * For another board (e.g. esc6288, different addresses) pass overrides and re-derive them first:
 *   ofd2000 -g <product.out> | grep -E 's_tx_in_flight|s_ints_enabled|s_rx|s_tx'
 *
 * Usage: dss.sh tools/flash/common/diag_tx_state.js <ccxml> <product.out> [txif=0x..] [ints=0x..] [rx=0x..] [tx=0x..]
 *   e.g. esc6288:  ... <ccxml> <out> txif=0xcfc0 ints=0xcfc1 rx=0xcfc2 tx=0xd086                    */
importPackage(Packages.com.ti.debug.engine.scripting);
importPackage(Packages.com.ti.ccstudio.scripting.environment);
importPackage(Packages.java.lang);
function p(s){ System.out.println(s); }
function w16(a){ return s.memory.readData(Memory.Page.DATA,a,16,1,false)[0]&0xFFFF; }
function w32(a){ var w=s.memory.readData(Memory.Page.DATA,a,16,2,false);
    return ((w[1]&0xFFFF)<<16)|(w[0]&0xFFFF); }
function hx(v){ return "0x"+(v>>>0).toString(16); }
/* launchxl_drv8305evm defaults; override with name=hex args (any order) for another board. */
var A={ TXIF:0xcf80, INTS:0xcf81, RX:0xcf82, TX:0xd046 }, overridden=false;
var ccxml=arguments[0], out=arguments[1];
for (var i=2;i<arguments.length;i++){ var kv=(""+arguments[i]).split("=");
    if(kv.length==2){ var k=kv[0].toLowerCase(), v=parseInt(kv[1],16);
        if(!isNaN(v)&&A.hasOwnProperty(k.toUpperCase())){ A[k.toUpperCase()]=v; overridden=true; } } }
var env=ScriptingEnvironment.instance(); env.setScriptTimeout(60000);
var server=env.getServer("DebugServer.1"); server.setConfig(ccxml);
var s=server.openSession("*","C28xx_CPU1"); s.target.connect(); s.memory.loadProgram(out);
var TXIF=A.TXIF, INTS=A.INTS, RX=A.RX, TX=A.TX;
p("  addrs: TXIF="+hx(TXIF)+" INTS="+hx(INTS)+" RX="+hx(RX)+" TX="+hx(TX)
  +(overridden?"  (overridden)":"  (launchxl defaults -- pass txif=/ints=/rx=/tx= for another board)"));
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
