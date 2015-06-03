import 'dart:async';
import 'dart:html';

class Client {
  static const Duration RECONNECT_DELAY = const Duration(milliseconds: 500);

  WebSocket ws;
  var hash;
  bool hasHash = false;
  //Constructor
  Client(button) {
    connect();
    if (button != "") querySelector(button).onClick.listen(sendToServer);
  }

  //Connect client to server
  void connect() {
    ws = new WebSocket('ws://127.0.0.1:7681');

    ws.onOpen.listen((e) {
      onConnected();
    });

    ws.onError.listen((e) {
      print("Error connecting to ws");
      reconnect();
    });
  }

  //When connected to server...
  void onConnected() {
    print('Connected');

    ws.onMessage.listen((e) {
      handleMessage(e.data);
    });
  }

  //Handle data from server
  void handleMessage(data) {
    print('något från servern');
    print(data);
    if (data.toString().contains("hash")) {
      print("got hash");
      hash = data.toString().substring(5);
      hasHash = true;
      print(hash);
    }

  }

  //Send data to server (start scanning process)
  void sendToServer(Event e) {
    ws.send('KINECTRGB');
  }

  //If error in connection, reconnect in 500 ms
  void reconnect() {
    new Timer(RECONNECT_DELAY, connect);
  }

  bool recievedHash() {
    return hasHash;
  }

}
