window.onscroll = function() {
  var header = document.querySelector('.bd-header');

  if (window.pageYOffset === 0) {
    header.classList.add('bd-header-no-shadow');
    header.classList.remove('bd-header-shadow');
  } else {
    header.classList.add('bd-header-shadow');
    header.classList.remove('bd-header-no-shadow');
  }
};
