//Medusa 3D-reconstruction


import 'dart:html';
//import 'package:three/three.dart';
import 'dart:async';


class Client
{
  static const Duration RECONNECT_DELAY = const Duration(milliseconds: 500);
  
  ButtonElement button = querySelector('#continueButton');
  WebSocket ws;
  
  //Constructor
  Client()
  {
    connect(); 
    button.onClick.listen(sendToServer);
  }
  
  //Connect client to server
  void connect()
  {
    ws = new WebSocket('ws://127.0.0.1:7681'); 
    
    ws.onOpen.listen((e)  
    {
      onConnected();    
    });
    
    ws.onError.listen((e) 
    {
      print("Error connecting to ws");
      reconnect();
    });
  }
  
  //When connected to server...
  void onConnected()
  {
    print('Connected');
    
    ws.onMessage.listen((e) 
    {
      handleMessage(e.data);
    });
  }
  
  //Handle data from server
  void handleMessage(data)
  { 
     print('något från servern');
  }
  
  //Send data to server (start scanning process)
  void sendToServer(Event e) 
  {
    ws.send('hej');
  }
  
  //If error in connection, reconnect in 500 ms
  void reconnect()
  {
    new Timer(RECONNECT_DELAY, connect);
  }
  
}

void main() 
{
  var client = new Client();
}



