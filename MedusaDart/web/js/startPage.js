//Medusa 3D-reconstruction
//This file handles the start up page
//classie.js is used to easily add and remove classes
 

(function( window )
{
  
  var infoMenu = document.querySelector( ".infoMenu" ),
      infoButton = document.querySelector( ".infoButton" ),
      startButton = document.querySelector( ".startButton" ),
      continueButton = document.querySelector( ".continueButton" ),
      body = document.body,
      mask = document.createElement("div"), 
      activeNav;
      
  mask.className = "mask";
  
  //Slide down the start button when it's pushed
  //Slide in instruction about the scanning process after the start button has been pushed
  startButton.addEventListener("click", function()
  {
    classie.add(body, "startButton-open");
    classie.add(body, "instructions-open");
    document.body.appendChild(mask);
    activeNav="startButton-open";
  });
  
  //Show information menu when the information button is pushed
  infoButton.addEventListener("click", function()
  {
    classie.add(body, "menu-open");
    document.body.appendChild(mask);
    activeNav="menu-open";
  
  });
  
  //Close the information menu when the close button is pushed
  [].slice.call(document.querySelectorAll(".closeButton")).forEach(function(el,i){
    el.addEventListener( "click", function(){
      classie.remove( body, activeNav );
      activeNav = "";
      document.body.removeChild(mask);
    } );
  });




})( window );