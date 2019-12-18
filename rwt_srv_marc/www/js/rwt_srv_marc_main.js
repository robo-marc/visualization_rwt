
$(function () {

  // subscribe topic
  var ros = new ROSLIB.Ros();
  ros.install_config_button("config-button");
  var serviceData = {};

  //テスト request

  function requestService() {
    ros.getSrvList(function (result) {
      var objList = [];
      $.each(result.message, function (index, msg) {
        var replace = msg.split('\'').join('\"');

        obj = JSON.parse(replace);
        objList.push(obj);
        // console.log(obj);
      })
      console.log(objList);
      // drawGrid(objList);
      selectService(objList);

      $("#service-form").submit(function (e) {
        e.preventDefault();
        var service_name = $("#service-select").val();
        if (sub) {
          console.log("unsubscribe");
          sub.unsubscribe();
        }
      });

      /コンソール表示用///////////////////////////////////////////////////////////////
      console.log(objList[58][0])  // テスト（58）
      var test = objList[58][0];   // テスト（58）

      // ros.serviceRequestTest2(test, function (type) {
      //   console.log(type.type);
      //   var item = type.type;
      //   if (item.indexOf('#') != -1) {
      //     item = type.type.replace(/#.*\n/g, '')
      //     // console.log(item);
      //     var code = item.slice(0, 1);
      //     while (code == '\n') {
      //       if (code == '\n') {
      //         item = item.replace('\n', '');
      //       }
      //       code = item.slice(0, 1);
      //     }
      //     console.log(item);
      //   }

      //   while (item.indexOf('=') != -1) {
      //     item = item.replace(/.*.=.*\n/, '')
      //     item = item.replace(/\n.*.=.*\n/g, '')
      //     while (code == '\n') {
      //       if (code == '\n') {
      //         item = item.replace('\n', '');
      //       }
      //       code = item.slice(0, 1);
      //     }

      //   }

      //   item = item.replace(/\n\n/g, '\n');
      //   console.log(item);

      // }, function (error) {
      //   console.log(error);
      // });

    }, function (mes) {
      console.log(mes);
    });
  }

  // ros events///////////////////////////////
  ros.on("connection", function () {
    ros.getTopics(function (topic_info) {
      var topics = topic_info.topics;
      topics.sort();
      //console.log("20：" + topics);
      $("#topic-select").append(_.map(topics, function (topic) {
        return '<option value="' + topic + '">' + topic + "</option>";
      }).join("\n"));
      $("#topic-select").change();
    });
  });

  ros.on("close", function () {
    $("#topic-select").empty();
  });
  ///////////////////////////////////////////

  requestService();

  // ★グリッドアイテム取得/////////////////////////////////////////
  var grid;
  var columns = [
    { id: "tree", name: "Tree", field: "tree", width: 200, minWidth: 20, maxWidth: 300 },
    { id: "type", name: "Type", field: "type", width: 200, minWidth: 20, maxWidth: 300 },
    { id: "path", name: "Path", field: "path", width: 200, minWidth: 20, maxWidth: 300 },
    { id: "remove", name: "Remove", field: "remove", width: 100, minWidth: 20, maxWidth: 200 }
  ];

  ///// グリッド部分のテスト///////////////////////////////////////
  // function drawGrid(objList) {
  //   var data = [];
  //   for (var i = 0; i < objList.length; i++) {
  //     var recordData = objList[i]
  //     for (var j = 0; j < recordData.length; j++) {

  //       data[j] = {
  //         tree: "",
  //         type: recordData[j],
  //         path: "",
  //         remove: ""
  //       };
  //     }

  //     grid = new Slick.Grid("#myGrid", data, columns);
  //   }
  // };

  ///////////////////////////////////////////////////////////////

  // serviceのリストを表示するリストボックス
  function selectService(objList) {
    var data = [];
    var serviceList = objList;
    for (var i = 0; i < objList.length; i++) {
      var recordData = objList[i]
      for (var j = 0; j < recordData.length; j++) {

        serviceStr = recordData[j]
        var str1 = serviceStr.substring(0, serviceStr.indexOf("/"));
        var str2 = serviceStr.substring(serviceStr.indexOf("/"));

        $("#service-select").append('<option value="' + str1 + '">' + str1 + "</option>");
        $("#message-select").append('<option value="' + str2 + '">' + str2 + "</option>");
      }
    };
    grid = new Slick.Grid("#myGrid", data, columns);
  };

  function serviceData() {
    selectService();

  }

  // // あとで追加
  //   $("#service-select").on("change", function () {
  //     var service_name = $("#service-select").val();
  //     $("#message-select").empty();
  //     $.each(serviceStr, function (value) {
  //       console.log(serviceStr)
  //       $("#message-select").append
  //         ('<option value="' + serviceStr + '">' + serviceStr + "</option>");
  //     })
  //   });

});
