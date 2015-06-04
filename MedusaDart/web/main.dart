//Medusa 3D-reconstruction
import 'wsclient.dart';
import 'dart:html';
import 'dart:core';
import 'dart:async';
import 'dart:js';

var timer;
var sTime;
var countDown = 3; //set the countdown to start from 3
Client client;
void main() {
  client = new Client('#continueButton');
  ButtonElement button = querySelector('#continueButton');
  button.onClick.listen(startCountdown);
}

void startCountdown(e) {
  HttpRequest.getString("countDown.html").then((html) {
    querySelector('#container').innerHtml = html;
  });
  sTime = new DateTime.now();
  timer = new Timer(const Duration(milliseconds: 500), updateTime);
}

//function to update the time from 3 to 0
void updateTime() {
  var cTime = new DateTime.now();
  var diff = cTime.difference(sTime).inMilliseconds; //difference between the time now and the start time(real time)
  var seconds = countDown - (diff / 1000).floor(); //3sec minus the time that have past(floor) is the time left to scanning
  if (seconds >= 0) {
    querySelector("#seconds").text = seconds;
    timer = new Timer(const Duration(milliseconds: 500), updateTime);
  } else {
    HttpRequest.getString("loadingPage.html").then((html) {
      querySelector('#container').innerHtml = html;
      initProgress();
      showProgress();
    });

  }
}

void initProgress() {
  var obj = context.callMethod('initProgress');
}

void showProgress() {
  if (client.hasHash) {
    var obj = context.callMethod('finish');
    Storage localStorage = window.localStorage;
    localStorage['hash'] = client.hash;
    window.location.replace("mainPage.html");
  } else {
    var obj = context.callMethod('increaseProgress');
    timer = new Timer(const Duration(milliseconds: 500), showProgress);
  }
}
