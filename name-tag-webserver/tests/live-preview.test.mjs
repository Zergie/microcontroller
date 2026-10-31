import { test } from 'node:test';
import assert from 'node:assert/strict';
import { frameCommand, createLivePreview } from '../mockups/live-preview.js';
const settle = async () => { for (let i=0;i<20;i++) await Promise.resolve(); };
function deferred() { let resolve,reject; const promise=new Promise((yes,no)=>{resolve=yes;reject=no;}); return {promise,resolve,reject}; }
const snapshot = active => ({active,fade_ms:1700,colors:Array(8).fill('#123456')});
function fixture(active=false) {
 const requests=[],waiting=[],timers=new Map(),statuses=[];
 let value=100,id=0;
 const live=createLivePreview({ sample:()=>({frame:Array.from({length:10},()=>[value,0,0]),glow:.5}),
  onStatus:message=>statuses.push(message),
  request(path,method='POST'){ requests.push({path,method});const d=deferred();waiting.push(d);return d.promise; },
  schedule(callback){timers.set(++id,callback);return id;},cancel(key){timers.delete(key);}
 });
 return {live,requests,waiting,timers,statuses,setValue(v){value=v;},
  async connect(){live.start();waiting[0].resolve(snapshot(active));await settle();},
  tick(){const [key,callback]=timers.entries().next().value;timers.delete(key);callback();}
 };
}
test('Ten concept pixels are interpolated into eight pads in controller order with glow applied',()=>{
 const frame=Array.from({length:10},(_,i)=>[i*28,0,0]);
 const hex=frameCommand(frame,.5).split('/').at(-1);
 assert.equal(hex.length,48);assert.equal(hex.slice(0,6),'7e0000');assert.equal(hex.slice(-6),'000000');
 assert.equal(frameCommand(Array(10).fill([300,-1,10])), '/api/pads/set/255/0/'+'ff000a'.repeat(8));
 assert.throws(()=>frameCommand([[NaN,0,0],[0,0,0]]));
});
test('Only Explore starts output; slow requests sample the latest frame without a backlog',async()=>{
 const f=fixture();assert.equal(f.requests.length,0);
 await f.connect();assert.equal(f.requests[0].method,'GET');assert.equal(f.requests.length,2);
 f.setValue(200); await settle(); assert.equal(f.requests.length,2); // A frame is still in flight.
 f.waiting[1].resolve({});await settle();assert.equal(f.timers.size,1);
 f.tick();await settle();assert.ok(f.requests[2].path.endsWith('640000'.repeat(8)));
 const stopped=f.live.stop();f.waiting[2].resolve({});await settle();
 assert.ok(f.requests[3].path.startsWith('/api/pads/set/0/1700/'));
 f.waiting[3].resolve({});await settle();assert.equal(f.requests[4].path,'/api/view/animations');
 f.waiting[4].resolve({});await stopped;assert.equal(f.timers.size,0);
});
test('Closing during connection never alters hardware',async()=>{
 const f=fixture();f.live.start();const stopped=f.live.stop();f.waiting[0].resolve(snapshot(false));await stopped;
 assert.equal(f.requests.length,1);
});
test('Closing an existing pad view restores colors/fade with no held fingers',async()=>{
 const f=fixture(true);await f.connect();f.waiting[1].resolve({});await settle();
 const stopped=f.live.stop();await settle();
 assert.equal(f.requests[2].path,'/api/pads/set/0/1700/'+'123456'.repeat(8));
 f.waiting[2].resolve({});await stopped;assert.equal(f.requests.length,3);
});
test('A timed-out frame still triggers restoration; a disconnected board stops output',async()=>{
 const f=fixture();await f.connect();f.waiting[1].reject(new Error('timeout'));await settle();
 assert.ok(f.requests[2].path.startsWith('/api/pads/set/0/1700/'));
 f.waiting[2].reject(new Error('offline'));await settle();
 assert.match(f.statuses.at(-1),/could not restore/);assert.equal(f.timers.size,0);
});
test('Reopening during restoration waits for it, then captures a fresh baseline',async()=>{
 const f=fixture();await f.connect();const stopped=f.live.stop();f.waiting[1].resolve({});await settle();
 f.live.start();assert.equal(f.requests.length,3); // Restoration is in flight.
 f.waiting[2].resolve({});await settle();f.waiting[3].resolve({});await settle();
 assert.equal(f.requests[4].path,'/api/pads.json');
 f.live.stop();f.waiting[4].resolve(snapshot(false));await stopped;
 assert.equal(f.requests.length,5);
});
test('Malformed board status never issues a write',async()=>{
 const f=fixture();f.live.start();f.waiting[0].resolve({colors:[]});await settle();
 assert.equal(f.requests.length,1);assert.match(f.statuses.at(-1),/valid eight-pad status/);
});
