kiloApp.controller('EditorCtrl', ['$scope', '$modal', '$http', '$timeout', 'focus', function($scope, $modal, $http, $timeout, focus) {
  $scope.content_template = "\
#include <kilolib.h>\n\
\n\
void setup() {\n\
    // put your setup code here, to be run only once\n\
}\n\
\n\
void loop() {\n\
    // put your main code here, to be run repeatedly\n\
}\n\
\n\
int main() {\n\
    // initialize hardware\n\
    kilo_init();\n\
    // start program\n\
    kilo_start(setup, loop);\n\
\n\
    return 0;\n\
}\n";
  $scope.active_file = null;
  $scope.available_files = [];
  $scope.has_changes = false;
  $scope.hex_autosave = true;
  $scope.file_autosave = true;

  $scope.$watch('dropbox_api.authenticated', function () {
    if ($scope.dropbox_api.authenticated)
      $scope.refreshDirectory();
  });

  $scope.confirm_delete = function (file) {
    var dialog_delete = $modal.open({
      controller: 'ConfirmDeleteCtrl',
      templateUrl: '/views/dialog_delete.html',
      resolve: {
        file: function () { return file; }
      }
    });

    dialog_delete.result.then(function (res) {
      if (file) {
        if (file  === $scope.active_file)
          $scope.active_file = null;
        $scope.refreshDirectory();
      }
    });
  };

  $scope.refreshDirectory = function () {
    $scope.loading_directory = true;
    $scope.dropbox_api.client.readdir('/', function (error, files, dir_stat, file_stats) {
      $scope.$apply(function () {
        if (error)
          $scape.dropbox_api.handleError(error);
        else {
          $scope.available_files = [];
          for (var i = 0; i<file_stats.length; i++) {
            if (file_stats[i].isFile && file_stats[i].name.indexOf('.hex', file_stats[i].name.length-4) === -1)
              $scope.available_files.push(file_stats[i].name);
          }
        }
        $scope.loading_directory = false;
      });
    });
  };

  $scope.startNew = function () {
    $scope.new_file = 'untitled.c';
    focus('newStarted');
  };

  $scope.cancelNew = function () {
    $scope.new_file = null;
  };

  $scope.finishNew = function () {
    if ($scope.available_files.indexOf($scope.new_file) !== -1) {
      $modal.open({
        controller: 'FileExistsCtrl',
        templateUrl: '/views/dialog_fileexists.html',
        resolve: {
          file: function () { return $scope.new_file; }
        }
      });
    } else {
      $scope.dropbox_api.client.writeFile($scope.new_file, $scope.content_template, function (error, stat) {
        $scope.$apply(function () {
          if (error)
            $scope.dropbox_api.handleError(error);
          else
            $scope.refreshDirectory();
        });
      });
      $scope.new_file = null;
    }
  };

  $scope.startRename = function (file) {
    $scope.rename_file =  file;
    $scope.rename_to = file;
    focus('renameStarted');
  };

  $scope.cancelRename = function () {
    $scope.rename_file = null;
  };

  $scope.finishRename = function () {
    if ($scope.rename_file === $scope.rename_to) {
      $scope.rename_file = null;
    } else if ($scope.available_files.indexOf($scope.rename_to) !== -1) {
      $modal.open({
        controller: 'FileExistsCtrl',
        templateUrl: '/views/dialog_fileexists.html',
        resolve: {
          file: function () { return $scope.rename_to; }
        }
      });
    } else {
      $scope.dropbox_api.client.move($scope.rename_file, $scope.rename_to, function (error) {
        $scope.$apply(function () {
          if (error)
            $scope.dropbox_api.handleError(error);
          else
            $scope.refreshDirectory();
        });
      });
      $scope.rename_file = null;
    }
  };

  $scope.hex_save = function () {
    var hex_name = $scope.active_file;
    if (hex_name.indexOf('.') !== -1)
      hex_name = hex_name.substr(0, hex_name.indexOf('.'));
    hex_name = hex_name + '.hex';
    $scope.hex_saving = true;
    $scope.dropbox_api.client.writeFile(hex_name, $scope.hex_content, function (error, stat) {
      $scope.$apply(function () {
        $scope.hex_saving = false;
        if (error)
          $scope.dropbox_api.handleError(error);
      });
    });
  };

  $scope.save = function () {
    $scope.editor.focus();
    $scope.saving = true;
    $scope.dropbox_api.client.writeFile($scope.active_file, $scope.editor.getValue(), function (error, stat) {
      $scope.$apply(function () {
        $scope.saving = false;
        if (error)
          $scope.dropbox_api.handleError(error);
        else
          $scope.has_changes = false;
      });
    });
  };

  $scope.edit = function (file) {
    $scope.opening = true;
    $scope.hex_content = null;
    $scope.dropbox_api.client.readFile(file, function (error, content, stat) {
      $scope.$apply(function () {
        $scope.opening = false;
        if (error)
          $scope.dropbox_api.handleError(error);
        else {
          $scope.active_file = file;
          $scope.editor.setValue(content);
        }
      });
    });
  };

  $scope.revert = function () {
    $scope.edit($scope.active_file);
  };

  $scope.compile = function () {
    $scope.editor.focus();
    $scope.compiling = true;
    $http({
      method: 'POST',
      url: 'https://www.kilobotics.com/api/compile/atmega328p',
      data: {content: $scope.editor.getValue()},
      headers: {'Content-Type': 'application/json'}
    }).success(function (data, status, headers, config) {
      $scope.compiling = false;
      if (data.hasOwnProperty('hex')) {
        $scope.hex_content = data.hex;
      } else
        $scope.hex_content = null;
      var annotations = [];
      if (data.annotations)
        annotations = data.annotations;
      $scope.editor.getSession().setAnnotations(annotations);
    }).error(function (data, status, headers, config) {
      $scope.hex_content = null;
      $scope.compiling = false;
      alert('error communicating with server.');
    })['finally'](function () {
      if ($scope.hex_autosave && $scope.hex_content)
        $scope.hex_save();
      if ($scope.file_autosave)
        $scope.save();
    });
  };

  $scope.set_theme = function (theme) {
    $scope.editor.setTheme('ace/theme/' + theme);
  };

  $scope.file_blob = function () {
    var blob = new Blob([$scope.editor.getValue()], {type: 'text/plain'});
    $scope.download_url = (window.URL || window.webkitURL).createObjectURL(blob);
  };

  $scope.hex_blob = function () {
    var blob = new Blob([$scope.hex_content], {type: 'text/plain'});
    $scope.hex_url = (window.URL || window.webkitURL).createObjectURL(blob);
  };

  $scope.signin = function () {
    $scope.dropbox_api.client.authenticate(function (error, c) {
      $scope.$apply(function() {
        if (error)
          $scope.dropbox_api.handleError(error);
        else
          $scope.dropbox_api.authenticated = true;
      });
    });
  };

  $scope.signout = function () {
    $scope.dropbox_api.client.signOut(function (error) {
      $scope.$apply(function() {
        if (error)
          $scope.dropbox_api.handleError(error);
        else
          $scope.dropbox_api.authenticated = false;
      });
    });
  };

  $scope.setEditor = function (editor) {
    $scope.editor = editor;
    // set theme
    editor.setTheme('ace/theme/xcode');
    // set language
    editor.getSession().setMode('ace/mode/c_cpp');
    // set editor options
    editor.setShowPrintMargin(false);
    // resize editor
    editor.resize();
    // record changes
    editor.on('change', function () {
      $timeout(function () {
        $scope.$apply(function () {
          $scope.has_changes = true;
        });
      });
    });
    // add compile command
    editor.commands.addCommand({
      name: 'compile',
      bindKey: {win: 'Ctrl-J',  mac: 'Command-J'},
      exec: function(editor) {
        $scope.$apply(function () {
          $scope.compile();
        });
      },
      readOnly: true
    });
    // add save command
    editor.commands.addCommand({
      name: 'save',
      bindKey: {win: 'Ctrl-S',  mac: 'Command-S'},
      exec: function(editor) {
        $scope.$apply(function () {
          $scope.save();
        });
      },
      readOnly: false
    });
  };
}]);

kiloApp.controller('NavCtrl', ['$scope', '$location', 'dropbox', function($scope, $location, dropbox) {
  $scope.isCollapsed = true;

  $scope.isActive = function (viewLocation) {
    return viewLocation === $location.path();
  };
}]);

kiloApp.controller('ConfirmDeleteCtrl', ['$scope', '$modalInstance', 'file', function ($scope, $modalInstance, file) {
  $scope.file = file;

  $scope.delete = function () {
    $scope.dropbox_api.client.remove($scope.file, function (error, stat) {
      if (error)
        $scope.dropbox_api.handleError(error);
      else{
        $modalInstance.close($scope.file);
      }
    });
  };

  $scope.close = function () {
    $modalInstance.dismiss();
  };
}]);

kiloApp.controller('FileExistsCtrl', ['$scope', '$modalInstance', 'file', function ($scope, $modalInstance, file) {
  $scope.file = file;

  $scope.close = function () {
    $modalInstance.dismiss();
  };
}]);

kiloApp.controller('WelcomeCtrl', [function () { }]);
kiloApp.controller('DownloadCtrl', [function () { }]);
kiloApp.controller('DocumentationCtrl', [function () { }]);
kiloApp.controller('LabsCtrl', [function () { }]);
