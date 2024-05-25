const { SerialPort } = require('serialport');
const express = require('express');
const http = require('http');
const socketIo = require('socket.io');
const cors = require('cors');

const PORT = process.env.PORT || 4000;

const SERIAL_PORT = 12;

const app = express();
app.use(cors());
const server = http.createServer(app);
const io = socketIo(server, {
    cors: {
        origin: true, // Allow requests from any origin
        methods: ["GET", "POST"] // Allow only GET and POST requests
    }
});

// Server start
server.listen(PORT, () => {
    console.log(`Server is running on port ${PORT}`);
});

let soc;
io.on('connection', (socket) => {
    console.log('Client connected');

    soc = socket;

    socket.on('disconnect', () => {
        console.log('Client disconnected');
    });
});

const baudRate = 115200;
const portInstance = new SerialPort({
    path: `COM${SERIAL_PORT}`,
    baudRate: baudRate
});

// To accumulate incoming data chunks
let buffer = '';

portInstance.on('data', (data) => {
    buffer += data.toString();
    let lines = buffer.split('\n');


    // Keep the last part in the buffer (it my be an incomplete line)
    buffer = lines.pop();


    lines.forEach((line) => {
        let trimmedLine = line.trim();
        if (trimmedLine) {


            if (soc) {
                soc.emit('distances', { distances: [trimmedLine]});
            }
        }
    });
});