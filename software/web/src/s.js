function G(id) {
    return document.getElementById(id);
}

var vector = {
    speed : 0,
    yaw   : 0,
};

var ws = {
    ws: null,
    status: false,
    error: false,
    updateInterval: null,
    init: function () {
		clearInterval(ws.updateInterval);
		try {
			ws.ws = new WebSocket(c.w);
				ws.ws.onopen = function() {
				ws.status = true;
			};
				ws.ws.onerror = function() {
				ws.status = false;
			};

			ws.updateInterval = setInterval(ws.update, 50);
		} catch(e) {
			clearInterval(ws.updateInterval);
			ws.status = false;
			ws.error = true;
			console.log(e);
		};
    },
    update: function(data) {
		if (ws.status) {
			ws.ws.send(packet.move());
		}
    }
};

var failsafe = {
	setFS: function () {
		vector.speed = 0;
		vector.yaw   = 0;
	}
};

var gui ={
    init: function () {
		gui.updateInterval = setInterval(gui.update, 100);
		document.addEventListener("visibilitychange", gui.onVisibilityChange);
    },
    update: function () {
		gui.showVector();
    },
    showVector: function () {
    },
    obj: {},
    updateInterval: null,
    onVisibilityChange: function () {
		if (document.visibilityState == "hidden") {
			failsafe.setFS();
		}
	}
};

var packet = {
	init: function () {
		packet.pMove = new ArrayBuffer(6);
		packet.vMove = new Uint8Array(packet.pMove);
	},
	_norm1: function (value) {
		return (value+1)*10000;
	},
    _uint16: function (view, num, offset) {
		view[offset]   = (num>>8)&255;
		view[offset+1] = num&255;
    },

    move: function () {
		packet.vMove[0] = 77;
		packet.vMove[1] = 1;
		packet._uint16(packet.vMove, packet._norm1(vector.speed), 2);
		packet._uint16(packet.vMove, packet._norm1(vector.yaw),   4);
		return packet.pMove;
	}
}

var onScreenGamepad = {
	obj: null,
	isEvent: false,
	deadband: 0.05,
	init: function () {
		onScreenGamepad.obj = G('joystick');
		onScreenGamepad.obj.addEventListener('mousedown', onScreenGamepad.eventStart);		
		onScreenGamepad.obj.addEventListener('touchstart', onScreenGamepad.eventStart);		
		
		onScreenGamepad.obj.addEventListener('mouseup', onScreenGamepad.eventFinish);
		onScreenGamepad.obj.addEventListener('mouseout', onScreenGamepad.eventFinish);
		onScreenGamepad.obj.addEventListener('mouseleave', onScreenGamepad.eventFinish);
		onScreenGamepad.obj.addEventListener('touchend', onScreenGamepad.eventFinish);
		onScreenGamepad.obj.addEventListener('touchcancel', onScreenGamepad.eventFinish);
		
		onScreenGamepad.obj.addEventListener('mousemove', onScreenGamepad.eventMove);
		onScreenGamepad.obj.addEventListener('touchmove', onScreenGamepad.eventMove);
	},
	eventStart() {
		onScreenGamepad.isEvent = true;
	},
	eventFinish() {
		onScreenGamepad.isEvent = false;
		vector.yaw   = 0;
		vector.speed = 0;
		onScreenGamepad.display(0, 0);
	},
	eventMove(event) {
		var x = (event.clientX || event.touches[0].clientX)/onScreenGamepad.obj.offsetWidth*2-1;
		var y = (event.clientY || event.touches[0].clientY)/onScreenGamepad.obj.offsetHeight*2-1;
		if (x >= -onScreenGamepad.deadband && x <= onScreenGamepad.deadband) x = 0;
		if (y >= -onScreenGamepad.deadband && y <= onScreenGamepad.deadband) y = 0;
		if (x > 1) x = 1;
		if (x < -1) x =-1;
		if (y > 1) y = 1;
		if (y < -1) y =-1;
		if (!onScreenGamepad.isEvent) {
			x = 0;
			y = 0;
		}

		vector.yaw   =  x;
		vector.speed = -y;

		onScreenGamepad.display(x, y);
	},
	display(x, y) {
		onScreenGamepad.obj.style.backgroundPosition = (x+1)/2*100 + '% ' + (y+1)/2*100 + '%';
	}
};

packet.init();
gui.init();
ws.init();
onScreenGamepad.init();
