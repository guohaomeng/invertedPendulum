/* 角度传感器串口 */
var Serial = {
  keepReading: true,
  port: '',
  reader: '',
  writer: '',
  receivedframe: [],
  begin: async function (Rate) {
    console.log('test');
    port = await navigator.serial.requestPort();
    await port.open({
      baudRate: Rate, // 波特率
      dataBits: 8,    // 每帧的数据位数(7或8)
      stopBits: 1,    // 停止位数(1或2)
      parity: 'none', // 校验模式，可以是none，偶数，奇数
      flowControl: 'none' // 流控模式(none或hardware)。
    });
    reader = port.readable.getReader();
    writer = port.writable.getWriter();
    console.log('角度传感器串口开启成功');
  },
  read: async function () {
    console.log('开始读取角度传感器串口');
    keepReading = true;//默认持续读取
    while (port.readable && keepReading) {
      try {
        while (true) {
          const { value, done } = await reader.read();
          if (done) {
            // 允许以后关闭串行端口。
            reader.releaseLock();
            break;
          }
          if (value.length >5 ) {
            //console.log(Uint8ArrayToString(value));
            /*** 处理数据值 ***/
            dealWithData(value);
          }
        }
      } catch (error) {
        // 处理非致命的读取错误。
        console.error(error);
      } finally {
        console.log(port.readable, keepReading);
        // 允许以后关闭串行端口。
        reader.releaseLock();
        writer.releaseLock();
      }
    }
  },
  write: async function(commandframe){
    
    await writer.write(commandframe);
    //port.close();
  },
  stop : async function(){
    keepReading = false;
    await reader.cancel();
    await port.close();
    console.log('角度传感器串口已关闭');
  }
}
/* SimpleFOC无刷电机串口 */
var motorSerial = {
  keepReading: true,
  port: '',
  reader: '',
  writer: '',
  receivedframe: [],
  begin: async function (Rate) {
    console.log('test');
    motorPort = await navigator.serial.requestPort();
    await motorPort.open({
      baudRate: Rate, // 波特率
      dataBits: 8,    // 每帧的数据位数(7或8)
      stopBits: 1,    // 停止位数(1或2)
      parity: 'none', // 校验模式，可以是none，偶数，奇数
      flowControl: 'none' // 流控模式(none或hardware)。
    });
    reader = motorPort.readable.getReader();
    writer = motorPort.writable.getWriter();
    console.log('电机串口开启成功');
  },
  readOnce : async function(){
    console.log('开始读取串口一次');
    keepReading = false;//不持续读取
    const { value, done } = await reader.read();
  },
  read: async function () {
    console.log('开始读取电机串口');
    keepReading = true;//默认持续读取
    while (motorPort.readable && keepReading) {
      try {
        while (true) {
          const { value, done } = await reader.read();
          if (done) {
            // 允许以后关闭串行端口。
            reader.releaseLock();
            break;
          }
          if (value.length > 5 ) {
            //console.log(Uint8ArrayToString(value));
            /*** 处理数据值 ***/
            dealWithData(value);
          }
        }
      } catch (error) {
        // 处理非致命的读取错误。
        console.error(error);
      } finally {
        console.log(port.readable, keepReading);
        // 允许以后关闭串行端口。
        reader.releaseLock();
        writer.releaseLock();
      }
    }
  },
  write: async function(commandframe){
    
    await writer.write(commandframe);
    //port.close();
  },
  stop : async function(){
    keepReading = false;
    await reader.cancel();
    await port.close();
    console.log('电机串口已关闭');
  }
}
/*** 外部串口接收数据处理函数 ***/
function dealWithData(value) {
  var tmp = Uint8ArrayToString(value);
  data = tmp.replace(/[^\d.]+/ig,"");  //去掉浮点数外部分
  document.getElementById('angleData').innerHTML = data + '°';
  console.log(data);
}

// hex转字符串
// 注意，仅支持英文
function Uint8ArrayToString(fileData) {
  var dataString = "";
  for (var i = 0; i < fileData.length; i++) {
    dataString += String.fromCharCode(fileData[i]);
  }
  return dataString
}

//字符串转hex
function stringToUint8Array(str) {
  var arr = [];
  for (var i = 0, j = str.length; i < j; ++i) {
    arr.push(str.charCodeAt(i));
  }
  var tmpUint8Array = new Uint8Array(arr);
  return tmpUint8Array
}
// 用法大概是这样
// await Serial.begin(115200);
// await Serial.read();