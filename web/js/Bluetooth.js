
function bluetooth() {
  //请求蓝牙设备
  navigator.bluetooth
    .requestDevice({ //显示附近所有设备
      acceptAllDevices: true 
    })
    .then((device) => { //控制台输出设备名称并连接
      console.log(device.name);
      return device.gatt.connect();
    })
    .then((server) => { //控制台输出设备服务
      console.log(server);
    })
    .catch((error) => { //错误捕捉
      console.error(error);
    });
}
class ESPmeng {

  constructor() {
    this.device = null;
    this.onDisconnected = this.onDisconnected.bind(this);
  }
  
  async request() {
    let options = {
      "filters": [{
        "name": "ESP32"
      }],
      "optionalServices": [0xFF02]
    };
    this.device = await navigator.bluetooth.requestDevice(options);
    if (!this.device) {
      throw "No device selected";
    }
    this.device.addEventListener('gattserverdisconnected', this.onDisconnected);
  }
  
  async connect() {
    if (!this.device) {
      return Promise.reject('Device is not connected.');
    }
    await this.device.gatt.connect();
  }
  
  async readData() {
    const service = await this.device.gatt.getPrimaryService(0xFF02);
    const characteristic = await service.getCharacteristic(0xFFFC);
    await characteristic.readValue();
  }

  async writeData(data) {
    const service = await this.device.gatt.getPrimaryService(0xFF02);
    const characteristic = await service.getCharacteristic(0xFFFC);
    await characteristic.writeValue(data);
  }

  disconnect() {
    if (!this.device) {
      return Promise.reject('Device is not connected.');
    }
    return this.device.gatt.disconnect();
  }

  onDisconnected() {
    console.log('Device is disconnected.');
  }
}

var eSPmeng = new ESPmeng();

document.querySelector('button').addEventListener('click', async event => {
  try {
    await eSPmeng.request();
    await eSPmeng.connect();
    /* Do something with eSPmeng... */
  } catch(error) {
    console.log(error);
  }
});