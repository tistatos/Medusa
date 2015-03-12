(function( window )
{

  var save = document.querySelector( ".save" ),
      body = document.body,
      mask = document.createElement("div"), 
      activeNav;
      
  mask.className = "mask";
  
  save.addEventListener("click", function()
  {
    classie.add(body, "qr-open");
    classie.add(body, "mail-open");
    document.body.appendChild(mask);
    activeNav="save-open";
  
  });

})( window );