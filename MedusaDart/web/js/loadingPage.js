//Medusa 3D-reconstruction
//This file handles the start up page
//classie.js is used to easily add and remove classes

(function( window )
{  
  //set up for the loadingbar
  window.onload = function onLoad() {
    var circle = new ProgressBar.Circle('#progress', {
      color: '#7EC0EA',
      trailColor: '#CBE6F7',
      strokeWidth: 7,
      duration: 2500,
      easing: 'easeInOut'
    });

    //start at 0
    circle.set(0);

    //go to 100% directly
    setTimeout(function() {
      circle.animate(1);
    }, 0); 

    //after 4000mm, go to mainPage.html
     setTimeout(function() {
      circle.animate(1);
      //fix, switch to mainPage to soon
      window.location.replace("mainPage.html"); 
    },4000);
  }; 

})( window );





