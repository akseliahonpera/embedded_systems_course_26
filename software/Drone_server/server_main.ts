
import dgram from 'node:dgram';
const server = dgram.createSocket('udp4');

server.on('error', (err) => {
  console.error(`server error:\n${err.stack}`);
  server.close();
});

server.on('message', (msg, rinfo) => {
  console.log(`server got: ${msg} from ${rinfo.address}:${rinfo.port}`);
  server.send("ack",rinfo.port, `${rinfo.address}`, (err) => {
    if(err){
      console.log("ack message sending failed due error");
    }
  }
);
});

server.on('listening', () => {
  const address = server.address();
  console.log(`server listening ${address.address}:${address.port}`);
});

server.bind(7777);