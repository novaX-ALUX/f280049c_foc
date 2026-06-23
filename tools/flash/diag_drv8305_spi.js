/*
 * diag_drv8305_spi.js - read DRV8305 registers over SPIA from DSS.
 *
 * Loads a lab image, waits at the SDK flagEnableSys dead-wait, asserts EN_GATE,
 * then uses SPIA registers directly. This avoids DSS target function calls,
 * which are unreliable in this CCS/DSS setup.
 *
 * Usage:
 *   dss.sh tools/flash/diag_drv8305_spi.js <ccxml> <lab.out>
 */
importPackage(Packages.com.ti.debug.engine.scripting);
importPackage(Packages.com.ti.ccstudio.scripting.environment);
importPackage(Packages.java.lang);

function p(s){ System.out.println(s); }
function num(s2,e2){ try { return Number(e2.evaluate(s2)); } catch(err){ return NaN; } }

var ccxml = arguments[0], out = arguments[1];
var env = ScriptingEnvironment.instance(); env.setScriptTimeout(60000);
var server = env.getServer("DebugServer.1"); server.setConfig(ccxml);
var s = server.openSession("*", "C28xx_CPU1");
s.target.connect(); s.memory.loadProgram(out);
var e = s.expression;

var SPIA = 0x6100;
var SPI_O_RXBUF = 0x7;
var SPI_O_TXBUF = 0x8;
var SPI_O_FFRX  = 0xB;
var SPI_FFRX_RXFFST_M = 0x1F00;
var SPI_FFRX_RXFFOVFCLR = 0x4000;
var SPI_FFRX_RXFIFORESET = 0x2000;

function rd16(addr){ return Number(s.memory.readData(Memory.Page.DATA, addr, 16, 1, false)[0]) & 0xFFFF; }
function wr16(addr, val){ s.memory.writeData(Memory.Page.DATA, addr, val & 0xFFFF, 16); }
function gateLow(){ try { wr16(0x7F0C, 0x80); } catch(x){} }

function resetRxFifo(){
    var r = rd16(SPIA + SPI_O_FFRX);
    wr16(SPIA + SPI_O_FFRX, (r & ~SPI_FFRX_RXFIFORESET) | SPI_FFRX_RXFFOVFCLR);
    r = rd16(SPIA + SPI_O_FFRX);
    wr16(SPIA + SPI_O_FFRX, r | SPI_FFRX_RXFIFORESET | SPI_FFRX_RXFFOVFCLR);
}

function rxCount(){
    return (rd16(SPIA + SPI_O_FFRX) & SPI_FFRX_RXFFST_M) >> 8;
}

function spiXfer(word){
    resetRxFifo();
    wr16(SPIA + SPI_O_TXBUF, word);
    for(var i=0; i<20000; i++){
        if(rxCount() > 0){
            return rd16(SPIA + SPI_O_RXBUF);
        }
    }
    return NaN;
}

function drvRead(addrShifted){
    var rwRead = 0x8000;
    var dataMask = 0x07FF;
    var word = rwRead | addrShifted;
    return spiXfer(word) & dataMask;
}

function hx(v){
    if(isNaN(v)) return "nan";
    var s2 = (v & 0xFFFF).toString(16).toUpperCase();
    while(s2.length < 4) s2 = "0" + s2;
    return "0x" + s2;
}

p("");
p("============ DRV8305 SPI register diagnostic ============");
s.target.runAsynch(); Thread.sleep(1200); s.target.halt();
var es0 = num("motorVars.flagEnableSys", e), hh = num("halHandle", e);
p("at dead-wait: flagEnableSys=" + es0 + "  halHandle=" + hh);
if(!(hh > 0) || es0 !== 0){
    p("FATAL: target is not at expected SDK dead-wait.");
    gateLow(); s.target.halt(); s.target.disconnect(); server.stop(); s.terminate();
    java.lang.System.exit(1);
}

p(">>> asserting EN_GATE (GPIO39) ...");
wr16(0x7F0A, 0x80);
s.target.runAsynch(); Thread.sleep(50); s.target.halt();
var g39 = ((rd16(0x7F08) & 0x80) !== 0) ? 1 : 0;
p("EN_GATE GPBDAT bit7 = " + g39);

// DRV8305 addresses already shifted left by 11, matching drv8305.h.
var regs = [
    ["STATUS_1",  1 << 11],
    ["STATUS_2",  2 << 11],
    ["STATUS_3",  3 << 11],
    ["STATUS_4",  4 << 11],
    ["CTRL_5",    5 << 11],
    ["CTRL_6",    6 << 11],
    ["CTRL_7",    7 << 11],
    ["CTRL_9",    9 << 11],
    ["CTRL_A",   10 << 11],
    ["CTRL_B",   11 << 11],
    ["CTRL_C",   12 << 11]
];

for(var i=0; i<regs.length; i++){
    var v = drvRead(regs[i][1]);
    p(regs[i][0] + " = " + hx(v) + " (" + v + ")");
}

p("=========================================================");
gateLow();
s.target.halt(); s.target.disconnect(); server.stop(); s.terminate();
