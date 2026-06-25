/* READ-ONLY: attach to the already-running target (NO loadProgram/reset), halt briefly, read the
 * can_bridge FIFO state by address, resume. Use to compare s_rx across host cansend bursts on ONE
 * continuous instance. Addresses (ofd2000 -g, esc0_node25/product.out):
 *   s_tx_in_flight 0xcf80, s_ints_enabled 0xcf81, s_rx 0xcf82, s_tx 0xd046.
 * Usage: dss.sh tools/flash/common/diag_rx_read.js <ccxml>  */
importPackage(Packages.com.ti.debug.engine.scripting);
importPackage(Packages.com.ti.ccstudio.scripting.environment);
importPackage(Packages.java.lang);
function p(s){ System.out.println(s); }
function w16(a){ return s.memory.readData(Memory.Page.DATA,a,16,1,false)[0]&0xFFFF; }
function w32(a){ var w=s.memory.readData(Memory.Page.DATA,a,16,2,false);
    return ((w[1]&0xFFFF)<<16)|(w[0]&0xFFFF); }
var ccxml=arguments[0];
var env=ScriptingEnvironment.instance(); env.setScriptTimeout(60000);
var server=env.getServer("DebugServer.1"); server.setConfig(ccxml);
var s=server.openSession("*","C28xx_CPU1"); s.target.connect();   /* NO loadProgram */
var TXIF=0xcf80, INTS=0xcf81, RX=0xcf82, TX=0xd046;
s.target.halt();
p("  s_ints_enabled="+w16(INTS)+" s_tx_in_flight="+w16(TXIF)
  +" | s_tx h/t="+w16(TX)+"/"+w16(TX+1)+" drop="+w32(TX+2)
  +" | s_rx h/t="+w16(RX)+"/"+w16(RX+1)+" drop="+w32(RX+2));
s.target.runAsynch(); s.target.disconnect(); server.stop(); s.terminate();
