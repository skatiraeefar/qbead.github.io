// Define mock arduino functions

var _ssw_t_page_open = new Date().getTime();
function millis() {
  return new Date().getTime() - _ssw_t_page_open;
}

// Define mock spinwheel class

class SpinWheelClass {
  constructor(container) {
    this._ssw_container = container;
    this._ssw_lLEDdiv = Array.from(container.getElementsByClassName("ssw-large-led"));
    this._ssw_sLEDdiv = Array.from(container.getElementsByClassName("ssw-small-led"));
    this._ssw_lLEDarray = new Uint8Array(this._ssw_lLEDdiv.length*3);
    this._ssw_sLEDarray = new Uint8Array(this._ssw_sLEDdiv.length*3);
    this._dx = 0.0;
    this._dy = 0.0;
    this._ddx = 0.0;
    this._ddy = 0.0;
    this.ax = 0.0;
    this.ay = 0.0;
    this.az = 1.0;
    this.gx = 0.0;
    this.gy = 0.0;
    this.gz = 0.0;
    this.mx = 0.0;
    this.my = 0.0;
    this.mz = 0.0;
  }

  begin() {
  }

  readIMU() {
    this.ax = this._ddx;
    this.ay = this._ddy;
  }

  drawLargeLEDFrame() {
    for (let i = 0; i<this._ssw_lLEDdiv.length; i++) {
      this._ssw_lLEDdiv[i].style.background = `rgb(${this._ssw_lLEDarray[3*i]},${this._ssw_lLEDarray[3*i+1]},${this._ssw_lLEDarray[3*i+2]})`;
    }
  }

  drawSmallLEDFrame() {
    for (let i = 0; i<this._ssw_sLEDdiv.length; i++) {
      this._ssw_sLEDdiv[i].style.background = `rgb(${this._ssw_sLEDarray[3*i]},${this._ssw_sLEDarray[3*i+1]},${this._ssw_sLEDarray[3*i+2]})`;
    }
  }

  drawFrame() {
    this.drawLargeLEDFrame();
    this.drawSmallLEDFrame();
  }
 
  setLargeLEDsUniform(r,g,b) {
    if (typeof g === 'undefined') {b = r & 0x0000FF; g = (r>>8) & 0x0000FF; r = (r>>16) & 0x0000FF;}
    for (let i = 0; i<this._ssw_lLEDdiv.length; i++) {
      this._ssw_lLEDarray[3*i  ] = r;
      this._ssw_lLEDarray[3*i+1] = g;
      this._ssw_lLEDarray[3*i+2] = b;
    }
  }

  setSmallLEDsUniform(r,g,b) {
    if (typeof g === 'undefined') {b = r & 0x0000FF; g = (r>>8) & 0x0000FF; r = (r>>16) & 0x0000FF;}
    for (let i = 0; i<this._ssw_sLEDdiv.length; i++) {
      this._ssw_sLEDarray[3*i  ] = r;
      this._ssw_sLEDarray[3*i+1] = g;
      this._ssw_sLEDarray[3*i+2] = b;
    }
  }

  setLargeLEDs(s,e,r,g,b) {
    if (typeof g === 'undefined') {b = r & 0x0000FF; g = (r>>8) & 0x0000FF; r = (r>>16) & 0x0000FF;}
    for (let i = Math.max(s,0); i<Math.min(e,this._ssw_lLEDdiv.length); i++) {
      this._ssw_lLEDarray[3*i  ] = r;
      this._ssw_lLEDarray[3*i+1] = g;
      this._ssw_lLEDarray[3*i+2] = b;
    }
  }

  setSmallLEDs(s,e,r,g,b) {
    if (typeof g === 'undefined') {b = r & 0x0000FF; g = (r>>8) & 0x0000FF; r = (r>>16) & 0x0000FF;}
    for (let i = Math.max(s,0); i<Math.min(e,this._ssw_sLEDdiv.length); i++) {
      this._ssw_sLEDarray[3*i  ] = r;
      this._ssw_sLEDarray[3*i+1] = g;
      this._ssw_sLEDarray[3*i+2] = b;
    }
  }

  setLargeLED(i, r,g,b) {
    if (typeof g === 'undefined') {b = r & 0x0000FF; g = (r>>8) & 0x0000FF; r = (r>>16) & 0x0000FF;}
    this._ssw_lLEDarray[3*i  ] = r;
    this._ssw_lLEDarray[3*i+1] = g;
    this._ssw_lLEDarray[3*i+2] = b;
  }

  setSmallLED(i, r,g,b) {
    if (typeof g === 'undefined') {b = r & 0x0000FF; g = (r>>8) & 0x0000FF; r = (r>>16) & 0x0000FF;}
    this._ssw_sLEDarray[3*i  ] = r;
    this._ssw_sLEDarray[3*i+1] = g;
    this._ssw_sLEDarray[3*i+2] = b;
  }

  clearAllLEDs() {
    this.setLargeLEDsUniform(0,0,0);
    this.setSmallLEDsUniform(0,0,0);
  }

  // TODO all of the set*** functions
}

function color(r, g, b) {
  r %= 256;
  g %= 256;
  b %= 256;
  return (r<<16)+(g<<8)+b;
}

function colorWheel(wheelPos) {
  wheelPos %= 256;
  wheelPos = 255 - wheelPos;
  if(wheelPos < 85) {
    return color(255 - wheelPos * 3, 0, wheelPos * 3);
  }
  if(wheelPos < 170) {
    wheelPos -= 85;
    return color(0, wheelPos * 3, 255 - wheelPos * 3);
  }
  wheelPos -= 170;
  return color(wheelPos * 3, 255 - wheelPos * 3, 0);
}

function triangularWave(x) {
  x %= 256;
  if (x>0x7f) {
    return (0xff-x)<<1;
  } else {
    return x<<1;
  }
}

// Define mock serial instance

var Serial = Object();
Serial.println = function() {console.log(String.prototype.concat.apply(Array.from(arguments).map(String)));};

// Processing functions

function get_original_code(container) {
  return Array.from(container.getElementsByClassName("ssw-codeblock"))
              .map(x => x.tagName!="TEXTAREA" ? x.innerHTML : x.value).join('');
}

var opdictionary = [
["&gt;", ">"],
["&lt;", "<"],
];
var dictionary = [
["void", "function"],
["int", "var"],
["float", "var"],
["abs", "Math.abs"],
["sqrt", "Math.sqrt"],
["pow", "Math.pow"],
];
function translate_code(code) {
  var base_code = " "+code+" ";
  dictionary.forEach(function (pair) {
    base_code = base_code.replace(RegExp('\\b'+pair[0]+'\\b','g'),
                                  pair[1]);
  });
  opdictionary.forEach(function (pair) {
    base_code = base_code.replace(pair[0],pair[1]);
  });
  var code = `
    var SpinWheel = this.SpinWheel;
    var button = this.querySelector('.ssw-bt-run');
    button.disabled = true;
    button.innerHTML = 'Running...';
    ${base_code}
    var c = 200;
    function _ssw_loop() {
      if (typeof setup !== 'undefined') {
        setup();
      }
      if (typeof loop !== 'undefined') {
        loop();
        c--;
        if (c>0) {
          setTimeout(_ssw_loop, 50);
        } else {
          button.disabled = false;
          button.innerHTML = 'Run for 10 seconds';
        }
      } else {
        button.disabled = false;
        button.innerHTML = 'Run for 10 seconds';
      }
    };
    _ssw_loop();
  `
  return code;
}

function show_in_debug(container) {
  container.getElementsByClassName('ssw-debug')[0].innerHTML = translate_code(get_original_code(container));
}

function run_sim(container) {
  Function(translate_code(get_original_code(container))).apply(container);
}

function unfinished_to_finished_html(unfinished_html) {
  return `
<div class="ssw-container">
<div class="ssw-code">
${unfinished_html}
<div class="ssw-codecontrols">
<button class="ssw-bt-run">Run for 10 seconds</button>
<button class="ssw-bt-debug">Debug</button>
<div class="ssw-debug">
</div>
</div>
</div>
<div class="ssw-vis-out">
<div class="ssw-vis">
<div>
<img src="/simspinwheel/spinwheel_invertgray.png">
<div class="ssw-large-led ssw-large-led0"></div>
<div class="ssw-large-led ssw-large-led1"></div>
<div class="ssw-large-led ssw-large-led2"></div>
<div class="ssw-large-led ssw-large-led3"></div>
<div class="ssw-large-led ssw-large-led4"></div>
<div class="ssw-large-led ssw-large-led5"></div>
<div class="ssw-large-led ssw-large-led6"></div>
<div class="ssw-large-led ssw-large-led7"></div>
<div class="ssw-small-led ssw-small-led0"></div>
<div class="ssw-small-led ssw-small-led1"></div>
<div class="ssw-small-led ssw-small-led2"></div>
<div class="ssw-small-led ssw-small-led3"></div>
<div class="ssw-small-led ssw-small-led4"></div>
<div class="ssw-small-led ssw-small-led5"></div>
<div class="ssw-small-led ssw-small-led6"></div>
<div class="ssw-small-led ssw-small-led7"></div>
<div class="ssw-small-led ssw-small-led8"></div>
<div class="ssw-small-led ssw-small-led9"></div>
<div class="ssw-small-led ssw-small-led10"></div>
<div class="ssw-small-led ssw-small-led11"></div>
</div>
</div>
</div>
</div>
`;
}

function prep_containers() {
  var unfinished_containers = document.querySelectorAll('.ssw-codecontent');
  unfinished_containers.forEach(function (x) {
    x.outerHTML = unfinished_to_finished_html(x.outerHTML);
  });
  var finished_containers = document.querySelectorAll('.ssw-container');
  finished_containers.forEach(function (x) {
    if (x.classList.contains('ssw-skip')) {return;}
    x.SpinWheel = new SpinWheelClass(x);
    var bt_run = x.querySelector('.ssw-bt-run');
    bt_run.onclick = function () {run_sim(x)};
    var bt_debug = x.querySelector('.ssw-bt-debug');
    bt_debug.onclick = function () {show_in_debug(x)};

    var textareas = x.querySelectorAll('textarea')
    textareas.forEach(function (text) {
      text.setAttribute("spellcheck","false");
      text.value = text.innerHTML.replace(/^\n+|\n+$/g, '');
      opdictionary.forEach(function (pair) {
        text.value = text.value.replace(pair[0], pair[1]);
      });
      function resize () {
          text.style.height = 'auto';
          text.style.height = (text.scrollHeight+3)+'px';
      }
      resize();
      text.resize = resize;
      text.addEventListener('input', resize, false);
    });
    setTimeout(function (){
      x.querySelector(".ssw-vis-out").style.width = x.querySelector(".ssw-vis").offsetWidth+'px';
      x.querySelector(".ssw-vis").style.position = "absolute";
      dragElement(x.querySelector(".ssw-vis"));
      },
      1000
    ); // needs time to reflow so that offsetWidth is not 0
  });
}

addEventListener('load', prep_containers, false);

// Dragging functions

function dragElement(elmnt) {
  var pos1 = 0, pos2 = 0, pos3 = 0, pos4 = 0;
  var t = 0.0, dx = 0.0, dy = 0.0, ddx = 0.0, ddy = 0.0;
  elmnt.onmousedown = dragMouseDown;
  var SpinWheel = elmnt.parentNode.parentNode.SpinWheel;

  function dragMouseDown(e) {
    e = e || window.event;
    e.preventDefault();
    elmnt.style.transform = "scale(3)";
    // get the mouse cursor position at startup:
    pos3 = e.clientX;
    pos4 = e.clientY;
    t = new Date().getTime();
    document.onmouseup = closeDragElement;
    document.onmousemove = elementDrag;
  }

  function elementDrag(e) {
    e = e || window.event;
    e.preventDefault();
    // calculate the new cursor position:
    pos1 = pos3 - e.clientX;
    pos2 = pos4 - e.clientY;
    var tnew = new Date().getTime();
    var dt = tnew-t;
    t = tnew;
    dx = pos1/dt*200;
    dy = pos2/dt*200;
    dx = isNaN(dx)?0:dx;
    dy = isNaN(dy)?0:dy;
    ddx = (dx-SpinWheel._dx)/dt;
    ddy = (dy-SpinWheel._dy)/dt;
    ddx = isNaN(ddx)?0:ddx;
    ddy = isNaN(ddy)?0:ddy;
    ddx = ddx>2?2:ddx<-2?-2:ddx;
    ddy = ddy>2?2:ddy<-2?-2:ddy;
    SpinWheel._dx = dx;
    SpinWheel._dy = dy;
    SpinWheel._ddx = 0.1*ddx + 0.9*SpinWheel._ddx;
    SpinWheel._ddy = 0.1*ddy + 0.9*SpinWheel._ddy;
    pos3 = e.clientX;
    pos4 = e.clientY;
    // set the element's new position:
    elmnt.style.top = (elmnt.offsetTop - pos2) + "px";
    elmnt.style.left = (elmnt.offsetLeft - pos1) + "px";
  }

  function closeDragElement() {
    elmnt.style.top = "auto";
    elmnt.style.left = "auto";
    elmnt.style.transform = "none";
    document.onmouseup = null;
    document.onmousemove = null;
  }
}
