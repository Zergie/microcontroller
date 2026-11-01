import {test} from 'node:test';
import assert from 'node:assert/strict';
import {createServer} from 'node:http';
import {readFile} from 'node:fs/promises';
import {createRequire} from 'node:module';
import {boardPins} from '../web/stm32-pins.js';
const require=createRequire(import.meta.url);
let chromium;
try { ({chromium}=require('playwright')); }
catch { ({chromium}=createRequire(new URL('../../rp2040-usb-webserver/package.json',import.meta.url))('playwright')); }

test('STM32 GPIO UI: drafts, reserved pins, PWM, mobile layout and errors',async()=>{
  const pins=Array.from({length:41},(_,pin)=>({pin,available:(pin<35 || pin>=39) && ![11,12,13,14,24].includes(pin),mode:'input',pull:'none',value:0,slew_rate:'slow'}));
  const commands=[];
  const server=createServer(async(req,res)=>{
    if(req.url==='/api/gpio/levels.json') {res.setHeader('Content-Type','application/json');res.end(JSON.stringify({levels:pins.map(p=>p.value)}));return;}
    if(req.url==='/api/gpio.json') {res.setHeader('Content-Type','application/json'); res.end(JSON.stringify({pins})); return;}
    if(req.method==='POST') {
      commands.push(req.url);
      if(req.url.includes('/pwm/')) {res.writeHead(409,{'Content-Type':'application/json'});res.end('{"error":"Timer frequency conflict."}');return;}
      const pin=Number(req.url.split('/')[3]); pins[pin]={...pins[pin],mode:'output',value:1};
      res.end('{"ok":true}');return;
    }
    try {
      const path=req.url==='/'?'index.html':req.url.slice(1);
      const data=await readFile(new URL('../web/'+path,import.meta.url));
      res.setHeader('Content-Type',path.endsWith('.js')?'text/javascript':path.endsWith('.css')?'text/css':'text/html');res.end(data);
    } catch {res.writeHead(404);res.end();}
  });
  await new Promise(r=>server.listen(0,'127.0.0.1',r));
  const browser=await chromium.launch({channel:process.platform==='win32'?'msedge':undefined});
  try {
    const page=await browser.newPage({viewport:{width:375,height:667}}),errors=[];
    page.on('pageerror',e=>errors.push(e.message));
    await page.goto(`http://127.0.0.1:${server.address().port}/`);
    await page.locator('#message').filter({hasText:'Connected'}).waitFor();
    assert.equal(await page.locator('[data-pin]').count(),39);
    assert.deepEqual(await page.locator('#left-edge .pin-label').allTextContents(),[
      'VCC','VCC','GND','GND','PB12','PB13','PB14','PB15','PC6','PA8','PA9','PA10',
      'PA11','PA12','PA15','PC10','PC11','PB3','PB4','PB5','PB6','PB7','PB8','PB9']);
    assert.deepEqual(await page.locator('#right-edge .pin-label').allTextContents(),[
      '3V3','3V3','GND','GND','PB10','PB11','PB2','V+','PB0','PB1','PA7','PC4',
      'PA5','PA6','PA3','PA4','PA1','PA2','NRST','PA0','PC14','PC15','VB','PC13']);
    assert.deepEqual(await page.locator('#debug-header .pin-label').allTextContents(),['3V3','DIO','CLK','GND']);
    const geometry=await page.evaluate(()=>{
      const box=selector=>document.querySelector(selector).getBoundingClientRect().toJSON();
      return {leftOuter:box('#left-edge .pin:nth-child(5)'),leftInner:box('#left-edge .pin:nth-child(6)'),
        rightInner:box('#right-edge .pin:nth-child(5)'),rightOuter:box('#right-edge .pin:nth-child(6)')};
    });
    assert(geometry.leftOuter.x<geometry.leftInner.x && geometry.leftInner.x<geometry.rightInner.x && geometry.rightInner.x<geometry.rightOuter.x);
    assert.equal(geometry.leftOuter.y,geometry.leftInner.y);
    assert.equal(geometry.leftOuter.y,geometry.rightOuter.y);
    await page.locator('[data-pin="39"]').click();
    assert.equal(await page.locator('#pin-form').evaluate(el=>el.closest('.editor').getAttribute('aria-label')),'Configure PC10');
    await page.locator('[data-pin="40"]').click();
    assert.equal(await page.locator('#pin-form').evaluate(el=>el.closest('.editor').getAttribute('aria-label')),'Configure PC11');
    await page.locator('#left-edge .pin').first().click();
    assert.equal(await page.locator('#apply-pin').isDisabled(),true);
    await page.locator('[data-pin="0"]').click();
    // A physical PC13 button edge must appear without clicking Refresh.
    pins[34].value=1;
    await page.waitForFunction(()=>document.querySelector('[data-pin="34"] .pin-level').textContent==='H',null,{timeout:2500});
    assert.equal(await page.locator('#key-state').textContent(),'H');
    pins[34].value=0;
    await page.waitForFunction(()=>document.querySelector('[data-pin="34"] .pin-level').textContent==='L',null,{timeout:2500});
    await page.locator('#pin-mode').selectOption('output');
    await page.locator('#output-level').selectOption('1');
    await page.locator('#output-level').dispatchEvent('input');
    pins[34].value=1;
    await page.waitForFunction(()=>document.querySelector('[data-pin="34"] .pin-level').textContent==='H',null,{timeout:2500});
    assert.equal(await page.locator('#output-level').inputValue(),'1');
    assert.equal(await page.locator('#apply-pin').isDisabled(),false);
    assert.equal(commands.length,0);
    await page.locator('[data-pin="4"]').click();
    await page.locator('[data-pin="0"]').click();
    assert.equal(await page.locator('#output-level').inputValue(),'1');
    await page.locator('#apply-pin').click();
    await page.waitForFunction(()=>document.querySelector('#apply-pin').disabled===false);
    assert.equal(commands[0],'/api/gpio/0/on/slew/slow');
    await page.locator('[data-pin="11"]').click(); assert.equal(await page.locator('#apply-pin').isDisabled(),true);
    await page.locator('[data-pin="4"]').click();
    assert.deepEqual(errors,[]);
    assert.equal(await page.locator('#pin-mode option[value="pwm"]').evaluate(el=>el.disabled),true);
    await page.locator('[data-pin="0"]').click();
    await page.locator('#pin-mode').selectOption('pwm');
    await page.locator('#apply-pin').click();
    await page.locator('#action-message').filter({hasText:'PWM conflict'}).waitFor();
    await page.locator('#pin-info').click(); assert.equal(await page.locator('#pin-info-dialog').isVisible(),true);
    await page.locator('#close-info').click();
    assert.equal(await page.evaluate(()=>document.documentElement.scrollWidth<=innerWidth),true);
    await page.reload();
    await page.locator('#message').filter({hasText:'Connected'}).waitFor();
    await page.screenshot({path:new URL('../build/ui-mobile.png',import.meta.url).pathname.replace(/^\/([A-Za-z]:)/,'$1')});
    await page.setViewportSize({width:1200,height:850});
    await page.screenshot({path:new URL('../build/ui-desktop.png',import.meta.url).pathname.replace(/^\/([A-Za-z]:)/,'$1')});
    await page.setViewportSize({width:375,height:1200});
    await page.locator('.page-scroll').evaluate(el=>{el.scrollTop=0;});
    await page.locator('.pinout').screenshot({path:new URL('../build/ui-board.png',import.meta.url).pathname.replace(/^\/([A-Za-z]:)/,'$1')});
    await page.setViewportSize({width:320,height:667});
    assert.equal(await page.evaluate(()=>document.documentElement.scrollWidth<=innerWidth),true);
    assert.deepEqual(errors,[]);
    assert.equal(boardPins.gpio(38).label,'PF1');
    assert.equal(boardPins.gpio(39).label,'PC10');
    assert.equal(boardPins.gpio(40).label,'PC11');
  } finally {await browser.close();server.closeAllConnections();await new Promise(r=>server.close(r));}
});
