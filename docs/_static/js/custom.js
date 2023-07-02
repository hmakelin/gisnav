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

// Handle dark mode mermaid diagrams
var observer = new MutationObserver(function(mutations) {
  const dark = document.documentElement.dataset.theme == 'dark';
  var mermaidTheme = dark ? 'dark' : 'default';
  mermaid.initialize({startOnLoad:true, theme: mermaidTheme});

  // Select all Mermaid divs
  var mermaidDivs = document.querySelectorAll('.mermaid');

  // Loop through each Mermaid div
  mermaidDivs.forEach(function(div) {
    // Retrieve the original Mermaid code from the 'data-mermaid' attribute
    var mermaidCode = div.getAttribute('data-mermaid');

    // Update the div's code, remove the 'data-processed' attribute
    div.innerHTML = mermaidCode;
    div.removeAttribute('data-processed');

    // Re-render the Mermaid diagram
    mermaid.init(undefined, div);
  });
});
observer.observe(document.documentElement, {attributes: true, attributeFilter: ['data-theme']});
