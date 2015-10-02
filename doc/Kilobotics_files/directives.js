kiloApp.directive('ace', [function () {
    return {
        restrict: 'A',
        scope: {
          callback: '&callbackFn',
        },
        link: function (scope, elem, attrs, ctrl) {
          var editor = ace.edit(elem[0]);
          scope.callback({arg: editor});
        }
    };
}]);

kiloApp.directive('ngEnter', function() {
  return function(scope, element, attrs) {
    element.bind("keydown keypress", function(event) {
      if(event.which === 13) {
        scope.$apply(function(){
          scope.$eval(attrs.ngEnter);
        });
        event.preventDefault();
      }
    });
  };
});

kiloApp.directive('ngEsc', function() {
  return function(scope, element, attrs) {
    element.bind("keydown keypress", function(event) {
      if(event.which === 27) {
        scope.$apply(function(){
          scope.$eval(attrs.ngEsc);
        });
        event.preventDefault();
      }
    });
  };
});

kiloApp.directive('focusOn', function() {
   return function(scope, elem, attr) {
      scope.$on('focusOn', function(e, name) {
        if (name === attr.focusOn) {
          elem[0].focus();
          elem[0].select();
        }
      });
   };
});
