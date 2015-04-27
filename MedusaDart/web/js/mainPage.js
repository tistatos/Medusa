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
      confirm = document.querySelector(".confirm");
  var clicked = false;
      
  mask.className = "mask";

  save.addEventListener("click", function()
  {
   // sweetAlert('Congratulations!', 'Your message has been successfully sent', 'success');
    swal({  title: "Maila mig min 3D-modell!",   
            text: "Skriv in din mailadress:",   
            type: "input",   showCancelButton: true,   
            closeOnConfirm: false,
            confirmButtonColor: "#7EC0EA",
            confirmButtonText: "Skicka",
            cancelButtonText: "Avbryt",  
            inputPlaceholder: "Din mail" }, 
            function(inputValue){   if (inputValue === false) return false;      
                                    if (inputValue === "") {     swal.showInputError("Du har inte fyllt i någon mailadress!");     
                                    return false   } 
                                      swal({
                                        title: "Modellen är skickad till", 
                                        text: inputValue,
                                        confirmButtonColor: "#7EC0EA", 
                                      }); 
                                    });
    });

  quit.addEventListener("click", function()
  {
    swal(
    {
      title: "Är du säker?",
      text: "Din modell kommer nu att kastas om du inte har sparat den!",
      showCancelButton: true,
      confirmButtonColor: "#7EC0EA",
      confirmButtonText: "OK",
      cancelButtonText: "Avbryt",
      closeOnConfirm: false
    },
      function()
      {
        window.location.replace("index.html");
      });
  });

  redo.addEventListener("click", function()
  {
      swal({
        title: "Är du säker på att du vill göra om modellen?",
        text: "Den nuvarande modellen kommer att kastas!",
        showCancelButton: true,
        confirmButtonColor: "#7EC0EA",
        confirmButtonText: "Gör om",
        cancelButtonText: "Avbryt",
        closeOnConfirm: false
      },
      function(){
        window.location.replace("countDown.html"); 
      });
  });

 
})( window );

//alert("Hello World!");