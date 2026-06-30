/* enc_probe.js - capture MT6701 raw SSI words + decode verdict via a breakpoint. READ-ONLY.
 * Breakpoints the return of MT6701_SSI_read() (mt6701_ssi.c) where w0/w1/frame24/f are live, and
 * reads them over a few hits. Tells dead-SPI (0x0000) vs MISO-stuck (0xffff) vs data-with-bad-CRC. */
importPackage(Packages.com.ti.debug.engine.scripting);
importPackage(Packages.com.ti.ccstudio.scripting.environment);
importPackage(Packages.java.lang);
function p(s){ System.out.println(s); }
function ev(nm){ try { return ""+e.evaluate(nm); } catch(err){ return "(opt-out)"; } }
var ccxml=arguments[0], out=arguments[1], BPLINE=arguments[2]?Number(arguments[2]):85;
var env=ScriptingEnvironment.instance(); env.setScriptTimeout(60000);
var server=env.getServer("DebugServer.1"); server.setConfig(ccxml);
var s=server.openSession("*","C28xx_CPU1");
s.target.connect(); s.memory.loadProgram(out);
var e=s.expression;
function fin(c){ try{s.target.halt();s.target.disconnect();}catch(x){} server.stop(); s.terminate(); java.lang.System.exit(c); }
p(""); p("======== MT6701 encoder raw-frame probe (bp @ mt6701_ssi.c:"+BPLINE+") ========");
var bp;
try { bp = s.breakpoint.add("mt6701_ssi.c", BPLINE); }
catch(err){ p("  breakpoint add failed: "+err); fin(1); }
for(var k=0;k<4;k++){
    try { s.target.run(); }     // runs until the breakpoint is hit (1 ms tick calls the read)
    catch(err){ p("  run/halt error: "+err); break; }
    var w0=ev("w0"), w1=ev("w1"), fr=ev("frame24");
    p("--- hit "+(k+1)+" ---");
    p("  raw SPI: w0="+w0+" w1="+w1+"  frame24="+fr);
    p("  decoded: angle="+ev("f.angle")+" mg=0x"+ev("f.mg")+
      "  crc_rx="+ev("f.crc_rx")+" crc_calc="+ev("f.crc_calc")+
      "  crc_ok="+ev("f.crc_ok")+" field_ok="+ev("f.field_ok")+" track_ok="+ev("f.track_ok"));
}
try { s.breakpoint.removeAll(); } catch(x){}
p("");
p("  read keys: w0/w1 all 0x0 => no SPI data (CLK/CSN/MISO dead or enc unpowered);");
p("             all 0xffff => MISO stuck high (no data / pullup); plausible bits but crc_ok=0 => alignment/HT0104/timing;");
p("             crc_ok=1 but field_ok/track_ok=0 => SPI good, magnet field weak / loss-of-track (placement).");
p("================================================================");
fin(0);
