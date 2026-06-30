/* can_es.js - read CANA error/status (LEC, error counters, bus state). READ-ONLY. */
importPackage(Packages.com.ti.debug.engine.scripting);
importPackage(Packages.com.ti.ccstudio.scripting.environment);
importPackage(Packages.java.lang);
function p(s){ System.out.println(s); }
function n(nm){ try { return Number(e.evaluate(nm)); } catch(err){ return NaN; } }
var ccxml=arguments[0], out=arguments[1];
var env=ScriptingEnvironment.instance(); env.setScriptTimeout(60000);
var server=env.getServer("DebugServer.1"); server.setConfig(ccxml);
var s=server.openSession("*","C28xx_CPU1");
s.target.connect(); s.memory.loadProgram(out);
var e=s.expression;
function done(c){ try{s.target.halt();s.target.disconnect();}catch(x){} server.stop(); s.terminate(); java.lang.System.exit(c); }
p(""); p("======== CANA error/status probe ========");
s.target.runAsynch(); Thread.sleep(1500); s.target.halt();
var LEC=["0 no-error","1 STUFF","2 FORM","3 ACK","4 BIT1","5 BIT0","6 CRC","7 no-event(none since last read)"];
function bits(v,lo,w){ return (v>>>lo)&((1<<w)-1); }
function reg(nm){ return n(nm)>>>0; }   // scalar read of the register through the symbol evaluator
function dump(tag){
    var es=reg("CanaRegs.CAN_ES"), errc=reg("CanaRegs.CAN_ERRC"), btr=reg("CanaRegs.CAN_BTR");
    var lec=bits(es,0,3);
    p("--- "+tag+" ---");
    p("  CAN_ES=0x"+es.toString(16)+"  LEC="+LEC[lec]+"  TXOK="+bits(es,3,1)+" RXOK="+bits(es,4,1)+
      "  EPASS="+bits(es,5,1)+" EWARN="+bits(es,6,1)+" BOFF="+bits(es,7,1)+" PER="+bits(es,8,1));
    p("  TEC(tx-err)="+bits(errc,0,8)+"  REC(rx-err)="+bits(errc,8,7)+"  RP="+bits(errc,15,1));
    var brp=bits(btr,0,6), sjw=bits(btr,6,2), t1=bits(btr,8,4), t2=bits(btr,12,3), brpe=bits(btr,16,4);
    var tq=1+(t1+1)+(t2+1), prescale=(brp+brpe*64)+1, bitrate=100e6/(prescale*tq);
    p("  CAN_BTR=0x"+btr.toString(16)+": BRP="+brp+" BRPE="+brpe+" TSEG1="+t1+" TSEG2="+t2+" SJW="+sjw+
      "  -> "+tq+" tq/bit, prescale "+prescale+"  => ~"+(bitrate/1e3).toFixed(1)+" kbit/s (expect ~1000 if 1 Mbit @100MHz CANclk)");
}
if(isNaN(n("CanaRegs.CAN_ES"))){ p("  CanaRegs.CAN_ES scalar read failed."); done(1); }
dump("read 1");
s.target.runAsynch(); Thread.sleep(1200); s.target.halt();
dump("read 2 (LEC re-armed after the read above)");
p("================================================================");
done(0);
