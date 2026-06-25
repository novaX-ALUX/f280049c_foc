/* READ-ONLY: attach to the already-running target (NO loadProgram/reset), halt briefly, read the
 * can_bridge FIFO state by address, resume. Use to compare s_rx across host cansend bursts on ONE
 * continuous instance.
 *
 * can_bridge's s_* are file-static and often stripped from the DSS expression evaluator, so this
 * reads them by ADDRESS. Addresses are PER-BUILD -- the defaults below are launchxl_drv8305evm
 * (esc0_node25/product.out). For another board (e.g. esc6288, which lands at different addresses)
 * pass overrides and re-derive them first:
 *   ofd2000 -g <product.out> | grep -E 's_tx_in_flight|s_ints_enabled|s_rx|s_tx'
 *
 * Usage: dss.sh tools/flash/common/diag_rx_read.js <ccxml> [txif=0x..] [ints=0x..] [rx=0x..] [tx=0x..]
 *   e.g. esc6288:  ... <ccxml> txif=0xcfc0 ints=0xcfc1 rx=0xcfc2 tx=0xd086                          */
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
var ccxml=arguments[0];
for (var i=1;i<arguments.length;i++){ var kv=(""+arguments[i]).split("=");
    if(kv.length==2){ var k=kv[0].toLowerCase(), v=parseInt(kv[1],16);
        if(!isNaN(v)&&A.hasOwnProperty(k.toUpperCase())){ A[k.toUpperCase()]=v; overridden=true; } } }
var env=ScriptingEnvironment.instance(); env.setScriptTimeout(60000);
var server=env.getServer("DebugServer.1"); server.setConfig(ccxml);
var s=server.openSession("*","C28xx_CPU1"); s.target.connect();   /* NO loadProgram */
var TXIF=A.TXIF, INTS=A.INTS, RX=A.RX, TX=A.TX;
p("  addrs: TXIF="+hx(TXIF)+" INTS="+hx(INTS)+" RX="+hx(RX)+" TX="+hx(TX)
  +(overridden?"  (overridden)":"  (launchxl defaults -- pass txif=/ints=/rx=/tx= for another board)"));
s.target.halt();
p("  s_ints_enabled="+w16(INTS)+" s_tx_in_flight="+w16(TXIF)
  +" | s_tx h/t="+w16(TX)+"/"+w16(TX+1)+" drop="+w32(TX+2)
  +" | s_rx h/t="+w16(RX)+"/"+w16(RX+1)+" drop="+w32(RX+2));
s.target.runAsynch(); s.target.disconnect(); server.stop(); s.terminate();
