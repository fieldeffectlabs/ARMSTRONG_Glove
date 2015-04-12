angular.module('gloveApp', [])
  .controller('gloveController', function($scope, $http, $timeout) {
    $scope.values = {};

    var getData = function () {
      console.log("GETTING DATA...");

      $http.get("http://127.0.0.1:30000")
      .success(function (data) {
        $scope.values = data;
      });
    }

    setInterval(getData, 100);
  });