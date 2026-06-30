/* check_is01_esc6288.js - is01 HAL bring-up SAFETY check on esc6288 (no spin, no arm). Confirms the
 * lab parks at flagEnableSys==0 with the gates OST-tripped, and reports raw ADC sanity. */
importPackage(Packages.com.ti.debug.engine.scripting);
importPackage(Packages.com.ti.ccstudio.scripting.environment);
importPackage(Packages.java.lang);
function p(s){ System.out.println(s); }
function num(nm){ try { return Number(e.evaluate(nm)); } catch(err){ return NaN; } }
function rd(a){ return s.memory.readData(Memory.Page.DATA,a,16,1,false)[0]&0xFFFF; }
var TZFLG=[0x4093,0x4193,0x4293], OST=0x4;
function ostStr(){ var r=""; for(var i=0;i<3;i++) r+=(((rd(TZFLG[i])&OST)!=0)?1:0)+(i<2?"/":""); return r; }
var ccxml=arguments[0], out=arguments[1];
var env=ScriptingEnvironment.instance(); env.setScriptTimeout(60000);
var server=env.getServer("DebugServer.1"); server.setConfig(ccxml);
var s=server.openSession("*","C28xx_CPU1");
s.target.connect(); s.memory.loadProgram(out);
var e=s.expression;
function done(c){ try{s.target.halt();s.target.disconnect();}catch(x){} server.stop(); s.terminate(); java.lang.System.exit(c); }
p(""); p("======== is01 HAL bring-up check (esc6288, no spin) ========");
s.target.runAsynch(); Thread.sleep(1200); s.target.halt();
var hh=num("halHandle"), es=num("motorVars.flagEnableSys");
p("  halHandle="+hh+" (expect nonzero)  flagEnableSys="+es+" (expect 0, parked at dead-wait)");
p("  EPWM OST="+ostStr()+" (expect 1/1/1 = gates safe-off)");
p("  raw ADC: IA(B15)="+rd(0x0B20)+" IB(A1)="+rd(0x0B00)+" IC(C2)="+rd(0x0B40)+" (expect ~2048)  Udc="+rd(0x0B01)+" (12V~480)");
var ok = (hh>0) && (es===0 || isNaN(es)) && (((rd(TZFLG[0])&OST)!=0)&&((rd(TZFLG[1])&OST)!=0)&&((rd(TZFLG[2])&OST)!=0));
p(ok ? ">>> is01 OK: HAL up, parked safe (OST set), gates off." : ">>> is01 CHECK: review the readings above.");
p("================================================================");
done(ok?0:1);
