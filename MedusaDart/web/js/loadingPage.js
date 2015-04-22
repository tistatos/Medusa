//Medusa 3D-reconstruction
//This file handles the start up page
//classie.js is used to easily add and remove classes

(function( window )
{  
  
  window.onload = function onLoad() {
    var circle = new ProgressBar.Circle('#progress', {
      color: '#777',
      trailColor: '#eee',
      strokeWidth: 9,
      duration: 2500,
      easing: 'easeInOut'
    });

    circle.set(0.05);

    setTimeout(function() {
      circle.animate(0.3);
    }, 1000);

    setTimeout(function() {
      circle.animate(0.4);
    }, 3500);

    setTimeout(function() {
      circle.animate(0.8);
    }, 5500);

    setTimeout(function() {
      circle.animate(0.9);
    }, 8000); 

     setTimeout(function() {
      circle.animate(1);
      //fix, switch to mainPage to soon
      window.location.replace("mainPage.html"); 
    }, 8000);
  }; 

})( window );





