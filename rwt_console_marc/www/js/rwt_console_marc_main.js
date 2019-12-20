function requiredFieldValidator(value) {
  if (value == null || value == undefined || !value.length) {
    return { valid: false, msg: "This is a required field" };
  } else {
    return { valid: true, msg: null };
  }
}


var TaskNameFormatter = function (row, cell, value, columnDef, dataContext) {
  if (value == null || value == undefined || dataContext === undefined) { return ""; }

  value = value.replace(/&/g, "&amp;").replace(/</g, "&lt;").replace(/>/g, "&gt;");
  var spacer = "<span style='display:inline-block;height:1px;width:" + (15 * dataContext["indent"]) + "px'></span>";
  var idx = dataView.getIdxById(dataContext.id);
  if (data[idx + 1] && data[idx + 1].indent > data[idx].indent) {
    if (dataContext._collapsed) {
      return spacer + " <span class='toggle expand'></span>&nbsp;" + value;
    } else {
      return spacer + " <span class='toggle collapse'></span>&nbsp;" + value;
    }
  } else {
    return spacer + " <span class='toggle'></span>&nbsp;" + value;
  }
};

var dataView;
var grid;
var data = [];
var columns = [
  { id: "#", name: "#", field: "#", minWidth: 60, editor: Slick.Editors.Text },
  { id: "Message", name: "Message", field: "Message", width: 220, editor: Slick.Editors.Text },
  { id: "Severity", name: "Severity", field: "Severity", editor: Slick.Editors.Text },
  { id: "Node", name: "Node", field: "Node", width: 80, editor: Slick.Editors.Text },
  { id: "Stamp", name: "Stamp", field: "Stamp", minWidth: 60, editor: Slick.Editors.Text },
  { id: "Topics", name: "Topics", field: "Topics", minWidth: 60, editor: Slick.Editors.Text },
  { id: "Location", name: "Location", width: 80, minWidth: 20, maxWidth: 80, field: "Location", editor: Slick.Editors.Text }
];

var options = {
  editable: true,
  enableAddRow: true,
  enableCellNavigation: true,
  asyncEditorLoading: false
};

var clear = function () {
  data = [];
  dataView.setItems(data);
  dataView.setFilter(myFilter);
  dataView.endUpdate();
}

function myFilter(item) {
  var percentCompleteThreshold = 0;
  var searchString = "";
  if (item["percentComplete"] < percentCompleteThreshold) {
    return false;
  }

  if (searchString != "" && item["title"].indexOf(searchString) == -1) {
    return false;
  }

  if (item.parent != null) {
    var parent = data[item.parent];

    while (parent) {
      if (parent._collapsed || (parent["percentComplete"] < percentCompleteThreshold) || (searchString != "" && parent["title"].indexOf(searchString) == -1)) {
        return false;
      }

      parent = data[parent.parent];
    }
  }

  return true;
}

function percentCompleteSort(a, b) {
  return a["percentComplete"] - b["percentComplete"];
}


$(function () {

  // subscribe topic
  var ros = new ROSLIB.Ros();
  ros.install_config_button("config-button");

  var list = [];
  var i = 1;

  function test() {
    var sub = null;
    sub = new ROSLIB.Topic({
      ros: ros,
      name: '/rosout_agg',
      messageType: 'rosgraph_msgs/Log'
    });
    sub.subscribe(function (msg) {
      var intTime = msg.header.stamp.secs;
      var d = new Date(intTime * 1000);
      var year = d.getFullYear();
      var month = d.getMonth() + 1;
      var day = d.getDate();
      var hour = ('0' + d.getHours()).slice(-2);
      var min = ('0' + d.getMinutes()).slice(-2);
      var sec = ('0' + d.getSeconds()).slice(-2);

      var time = hour + ":" + min + ":" + sec + "." + msg.header.stamp.nsecs + "(" + year + "-" + month + "-" + day + ")";
      var mes = msg.msg;
      var severilty = msg.level;
      var node = msg.name;
      var stamp = time;
      var topics = msg.topics.join(',');
      var location = msg.file + ":" + msg.function + ":" + msg.line;
      var levelName = null;

      if (severilty == '1') {
        levelName = 'DEBUG';
      } else if (severilty == '2') {
        levelName = 'INFO';
      } else if (severilty == '4') {
        levelName = 'WARN';
      } else if (severilty == '8') {
        levelName = 'ERROR';
      } else if (severilty == '16') {
        levelName = 'FATAL';
      }

      var associationList = { '#': '#' + i, Message: mes, Severity: levelName, Node: node, Stamp: stamp, Topics: topics, Location: location };
      // console.log(associationList);
      list.push(associationList);
      sample();
      i++;
    });
  };

  document.getElementById('clearButton').onclick = function () {
    list = [];
    clear();
    i = 1;
    sample();
  };

  document.getElementById('downloadCSV').onclick = function (e) {
    // e.preventDefault();
    var arr = []
    _.each(list, function (value, index) {
      arr.push(Object.keys(value).map(function (key) {
        return value[key]
      })
      );
    });
    var csvData;
    _.each(arr, function (value, index) {
      let row = value.join('  ');
      csvData += row + '\n';
    });
    var blob = new Blob([csvData], { "type": "text/plain" });
    if (window.navigator.msSaveBlob) {
      window.navigator.msSaveBlob(blob, "test.csv");
      window.navigator.msSaveOrOpenBlob(blob, "test.csv");
    } else {
      document.getElementById("downloadCSV").href = window.URL.createObjectURL(blob);
    }
    // var encodedUri = encodeURI(csvData);
    // var link = document.getElementById("downloadCSV");
    // link.setAttribute("href", encodedUri);
    // link.setAttribute("download", "csvdata.csv");
  };

  var sample = function () {
    var indent = 0;
    var parents = [];
    // prepare the data
    _.each(list, function (msg, index) {
      var d = (data[index] = {});
      var parent;

      if (Math.random() > 0.8 && index > 0) {
        indent++;
        parents.push(index - 1);
      } else if (Math.random() < 0.3 && indent > 0) {
        indent--;
        parents.pop();
      }

      if (parents.length > 0) {
        parent = parents[parents.length - 1];
      } else {
        parent = null;
      }
      d["id"] = "id_" + index;
      d["indent"] = indent;
      d["parent"] = parent;
      d["#"] = msg['#'];
      d["Message"] = msg.Message;
      d["Severity"] = msg.Severity;
      d["Node"] = msg.Node;
      d["Stamp"] = msg.Stamp;
      d["Topics"] = msg.Topics;
      d["Location"] = msg.Location;
    });


    // initialize the model
    dataView = new Slick.Data.DataView({ inlineFilters: true });
    dataView.beginUpdate();
    dataView.setItems(data);
    dataView.setFilter(myFilter);
    dataView.endUpdate();


    // initialize the grid
    grid = new Slick.Grid("#myGrid", dataView, columns, options);

    grid.onCellChange.subscribe(function (e, args) {
      dataView.updateItem(args.item.id, args.item);
    });

    // grid.onAddNewRow.subscribe(function (e, args) {
    //   var item = {
    //     "id": "new_" + (Math.round(Math.random() * 10000)),
    //     "indent": 0,
    //     "title": "New task",
    //     "duration": "1 day",
    //     "percentComplete": 0,
    //     "start": "01/01/2009",
    //     "finish": "01/01/2009",
    //     "effortDriven": false
    //   };
    //   $.extend(item, args.item);
    //   dataView.addItem(item);
    // });

    // grid.onClick.subscribe(function (e, args) {
    //   if ($(e.target).hasClass("toggle")) {
    //     var item = dataView.getItem(args.row);
    //     if (item) {
    //       if (!item._collapsed) {
    //         item._collapsed = true;
    //       } else {
    //         item._collapsed = false;
    //       }

    //       dataView.updateItem(item.id, item);
    //     }
    //     e.stopImmediatePropagation();
    //   }
    // });


    // wire up model events to drive the grid
    dataView.onRowCountChanged.subscribe(function (e, args) {
      grid.updateRowCount();
      grid.render();
    });

    dataView.onRowsChanged.subscribe(function (e, args) {
      grid.invalidateRows(args.rows);
      grid.render();
    });


    var h_runfilters = null;

    // wire up the slider to apply the filter to the model
    // $("#pcSlider").slider({
    //   "range": "min",
    //   "slide": function (event, ui) {
    //     Slick.GlobalEditorLock.cancelCurrentEdit();

    //     if (percentCompleteThreshold != ui.value) {
    //       window.clearTimeout(h_runfilters);
    //       h_runfilters = window.setTimeout(dataView.refresh, 10);
    //       percentCompleteThreshold = ui.value;
    //     }
    //   }
    // });


    // wire up the search textbox to apply the filter to the model
    $("#txtSearch").keyup(function (e) {
      Slick.GlobalEditorLock.cancelCurrentEdit();

      // clear on Esc
      if (e.which == 27) {
        this.value = "";
      }

      searchString = this.value;
      dataView.refresh();
    })
  };

  sample();
  test();
});