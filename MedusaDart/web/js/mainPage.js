(function( window )
{

  var save = document.querySelector( ".save" ),
      //popupQR = document.querySelector( ".qr" ),
      popupMail = document.querySelector( ".mail" ),
      redo = document.querySelector("#reDo"),
      body = document.body,
      mask = document.createElement("div"), 
      activeNav,
      quit = document.querySelector("#quit");
  var clicked = false;
      
  mask.className = "mask";

    save.addEventListener("click", function()
  {
   // sweetAlert('Congratulations!', 'Your message has been successfully sent', 'success');
    swal({  title: "Maila mig min 3D-modell!",   
            text: "Skriv in din mailadress:",   
            type: "input",   showCancelButton: true,   
            closeOnConfirm: false,
            confirmButtonText: "Skicka",
            cancelButtonText: "Avbryt",  
            inputPlaceholder: "Din mail" }, 
            function(inputValue){   if (inputValue === false) return false;      
                                    if (inputValue === "") {     swal.showInputError("Du har inte fyllt i någon mailadress!");     
                                    return false   }      swal("Modellen är skickad till", inputValue); });
    });

  quit.addEventListener("click", function()
  {
    swal({
      title: "Är du säker?",
      text: "Din modell kommer nu att kastas!",
      type: "warning",
      showCancelButton: true,
      confirmButtonColor: "#DD6B55",
      confirmButtonText: "Ja, kasta den!",
      cancelButtonText: "Avbryt",
      closeOnConfirm: false
    },
    function(){
      swal("Kastad!", "Din modell är nu borta.. för alltid!", "success");
    });
  });
  
 /* quit.addEventListener("click", function()
  {
      swal({
        title: "Är du säker på att du vill avsluta?",
        text: "Den nuvarande modellen kommer att kastas!",
        showCancelButton: true,
        confirmButtonColor: "#DD6B55",
        confirmButtonText: "Avsluta",
        cancelButtonText: "Avbryt",
        closeOnConfirm: false
      },
      function(){
        window.location.replace("index.html");
      });
   });*/

  redo.addEventListener("click", function()
  {
      swal({
        title: "Är du säker på att du vill göra om modellen?",
        text: "Den nuvarande modellen kommer att kastas!",
        showCancelButton: true,
        confirmButtonColor: "#DD6B55",
        confirmButtonText: "Gör om!",
        cancelButtonText: "Avbryt",
        closeOnConfirm: false
      },
      function(){
        window.location.replace("countDown.html"); 
      });
  });

 
})( window );

//alert("Hello World!");