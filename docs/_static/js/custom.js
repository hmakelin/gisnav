window.onscroll = function() {
  var header = document.querySelector('.bd-header');

  if (window.pageYOffset === 0) {
    header.classList.add('bd-header-no-shadow');
    header.classList.remove('bd-header-shadow');
  } else {
    header.classList.add('bd-header-shadow');
    header.classList.remove('bd-header-no-shadow');
  }

  if (window.pageYOffset === 0) {
    document.body.classList.add('bounce');

    // Get elements with class 'skip-link' and hide them
    var skipLinks = document.getElementsByClassName('skip-link');
    for (var i = 0; i < skipLinks.length; i++) {
      skipLinks[i].style.display = 'none';
    }

    setTimeout(function() {
      document.body.classList.remove('bounce');

      // Show the 'skip-link' elements again
      for (var i = 0; i < skipLinks.length; i++) {
        skipLinks[i].style.display = 'block';
      }
    }, 500);
  }
};
