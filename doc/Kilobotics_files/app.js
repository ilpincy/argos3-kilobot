var kiloApp = angular.module('kiloApp', ['ui.bootstrap', 'ngRoute']);

kiloApp.config(['$routeProvider', '$locationProvider', '$compileProvider', function ($routeProvider, $locationProvider, $compileProvider) {
  $routeProvider.when('/', {templateUrl: '/views/welcome.html', controller: 'WelcomeCtrl'});
  $routeProvider.when('/documentation', {templateUrl: '/views/documentation.html', controller: 'DocumentationCtrl'});
  $routeProvider.when('/labs', {templateUrl: '/views/labs.html', controller: 'LabsCtrl'});
  $routeProvider.when('/editor', {templateUrl: '/views/editor.html?v2', controller: 'EditorCtrl'});
  $routeProvider.when('/download', {templateUrl: '/views/download.html?v4', controller: 'DownloadCtrl'});

  $locationProvider.html5Mode(true).hashPrefix('!');
  $compileProvider.aHrefSanitizationWhitelist(/^\s*(https?|ftp|file|blob):|data:image\//);
}]);
