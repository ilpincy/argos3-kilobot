kiloApp.factory('dropbox', ['$rootScope', '$timeout', function ($rootScope, $timeout) {
  var dropbox_api = {
    authenticated: null,
    client: new Dropbox.Client({key: "2rgw0nwtvj1lodj"})};

  dropbox_api.client.authDriver(new Dropbox.AuthDriver.Popup({
    /* receiverUrl: "http://localhost:8000/dropbox_complete.html"})); */
    receiverUrl: "https://www.kilobotics.com/dropbox_complete.html"}));

  dropbox_api.handleError = function (error) {
    if (error.hasOwnProperty('response') && error.response.hasOwnProperty('error'))
      alert(error.response.error);
    else
      alert(error);
  };

  function do_authenticate(error, client) {
    $rootScope.$apply(function () {
      if (error || !client.isAuthenticated()) 
        dropbox_api.authenticated = false;
      else
        dropbox_api.authenticated = true;
    });
  }

  dropbox_api.client.authenticate({interactive: false}, function (error, client) {
    setTimeout(function () { do_authenticate(error, client); }, 100);
  });

  $rootScope.dropbox_api = dropbox_api;

  return dropbox_api;
}]);

kiloApp.factory('focus', ['$rootScope', '$timeout', function ($rootScope, $timeout) {
  return function(name) {
    $timeout(function (){
      $rootScope.$broadcast('focusOn', name);
    });
  }
}]);
