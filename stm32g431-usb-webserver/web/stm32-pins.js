// WeAct QFN48 top view, USB at top: reference rotated 90 degrees clockwise.
// Stable API indices; PC10/PC11 are appended.
const names = [...Array.from({length:16}, (_, i) => `PA${i}`),
  ...Array.from({length:16}, (_, i) => `PB${i}`), 'PC4','PC6','PC13','PC14','PC15','PF0','PF1','PC10','PC11'];
const pwm = {PA0:'TIM2 CH1', PA1:'TIM2 CH2', PA2:'TIM2 CH3', PA3:'TIM2 CH4',
  PA6:'TIM3 CH1', PA7:'TIM3 CH2', PB0:'TIM3 CH3', PB1:'TIM3 CH4',
  PB6:'TIM4 CH1', PB7:'TIM4 CH2', PB9:'TIM4 CH4'};
const reserved = {PA11:'USB D−', PA12:'USB D+', PA13:'SWDIO', PA14:'SWCLK', PB8:'BOOT0',
  PC14:'LSE oscillator',PC15:'LSE oscillator',PF0:'HSE oscillator',PF1:'HSE oscillator'};
const pins = names.map((label, gpio) => ({key:`gpio-${gpio}`, gpio, physical:null,
  kind:'gpio', label, adc:null, pwm:pwm[label],
  note: label==='PC13' ? 'WeAct C13 user button: active HIGH. Use Input – Pull-down (startup default). Released = L, pressed = H. Live levels refresh automatically.' : reserved[label] ? `Reserved: ${reserved[label]}.` :
    'Check your board schematic before driving this pin. ADC is not implemented. Settings reset at power-up.',
  legend:[{category:'gpio', title:label, detail:reserved[label] || 'Digital input/output · pull-up/down · open drain', supported:!reserved[label]},
    ...(pwm[label] ? [{category:'gpio',title:'PWM',detail:pwm[label],supported:true}] : [])]
}));
const byName = new Map(pins.map(pin=>[pin.label,pin]));
const passive = {
  VCC:['power','VCC board supply rail'], '3V3':['power','3.3 V rail'],
  GND:['ground','Ground'], 'V+':['power','VREF+ · analog reference'],
  VB:['power','VBAT · backup supply'], NRST:['control','Reset input · PG10']
};
function contact(label, physical) {
  const pin=byName.get(label);
  if(pin) return {...pin,physical};
  const [kind,detail]=passive[label];
  return {key:physical,gpio:null,physical,label,kind,note:'Information only.',
    legend:[{category:kind==='control'?'system':kind,title:label,detail,supported:true}]};
}
// Pairs are [outer, inner] on the left and [inner, outer] on the right.
const leftPairs=[
  ['VCC','VCC'],['GND','GND'],['PB12','PB13'],['PB14','PB15'],
  ['PC6','PA8'],['PA9','PA10'],['PA11','PA12'],['PA15','PC10'],
  ['PC11','PB3'],['PB4','PB5'],['PB6','PB7'],['PB8','PB9']
];
const rightPairs=[
  ['3V3','3V3'],['GND','GND'],['PB10','PB11'],['PB2','V+'],
  ['PB0','PB1'],['PA7','PC4'],['PA5','PA6'],['PA3','PA4'],
  ['PA1','PA2'],['NRST','PA0'],['PC14','PC15'],['VB','PC13']
];
function header(pairs, side) {
  return pairs.flatMap((pair,row)=>pair.map((label,col)=>contact(label,
    `${side} ${col===(side==='left'?0:1)?'outer':'inner'}, row ${row+1} from USB`)));
}
export const boardPins = Object.freeze({left:header(leftPairs,'left'),right:header(rightPairs,'right'),
  debug:['3V3','PA13','PA14','GND'].map((name,i)=>({...contact(name,`SWD ${i+1} from left`),label:({PA13:'DIO',PA14:'CLK'})[name] || name})),
  gpio:number=>pins[number]});
