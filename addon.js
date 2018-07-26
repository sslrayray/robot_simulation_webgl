var addon = require('bindings')('addon');

var boxes = addon('0', '0', '0', '0', '0', '0');
//var obj2 = addon('world', 'world2');
//console.log(obj1); // 'hello world'


var app = require('express')();
var server = require('http').Server(app);
var io = require('socket.io')(server);

server.listen(80);

app.get('/', function (req, res) {
  res.sendfile(__dirname + '/index.html');
});

io.on('connection', function (socket) {
  socket.emit('news', boxes);
  //socket.emit('news', { hello: 'world' });
  socket.on('my other event', function (data) {
	boxes = addon(data['axis1'], data['axis2'], data['axis3'], data['axis4'], data['axis5'], data['axis6']);
	//console.log(boxes);
	socket.emit('news', boxes);
    //console.log(data['axis1']+','+data['axis2']+','+data['axis3']+','+data['axis4']+','+data['axis5']+','+data['axis6']);
  //console.log(data['axis2']);
  });
});
    