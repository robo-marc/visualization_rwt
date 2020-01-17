var FilterUtils = {

  ////////////////////////////////////////
  // public

  getMessageFilterHtml: function (conditionType) {
    var contentHtml = ''
      + '<div class="parts-set textbox"><input type="text" class="message-value"placeholder="Message Text"></div>'
      + '<div class="parts-set checkbox"><input type="checkbox"><label for="">Regex</label></div>'
      ;
    return FilterUtils.getCommonHtml('message', 'Message', conditionType, contentHtml);
  },

  getSeverityFilterHtml: function (conditionType) {
    var contentHtml = ''
      + '<ul class="chips">'
      + '<li><a href class="severities-select-button" data-value="Debug">Debug</a></li>'
      + '<li><a href class="severities-select-button" data-value="Info">Info</a></li>'
      + '<li><a href class="severities-select-button" data-value="Warn">Warn</a></li>'
      + '<li><a href class="severities-select-button" data-value="Error">Error</a></li>'
      + '<li><a href class="severities-select-button" data-value="Fatal">Fatal</a></li>'
      + '</ul>'
      ;
    return FilterUtils.getCommonHtml('severities', 'Severities', conditionType, contentHtml);
  },

  getNodesFilterHtml: function (conditionType, list) {
    var contentHtml = '<select class="node-value" type="select">';
    _.each(list, function (item, index) {
      contentHtml += '<option value="' + item + '">' + item + '</option>';
    });
    contentHtml += '</select>';

    return FilterUtils.getCommonHtml('node', 'Node', conditionType, contentHtml);
  },

  getStampFilterHtml: function (conditionType) {
    var now = new Date();
    var dateStr = FilterUtils.getDateStr(now);
    var timeStr = FilterUtils.getTimeStr(now);
    var contentHtml = ''
      + '<div class="parts-set stamp"><input type="time" class="time begin" step="0.01" value="' + timeStr + '"><input type="date" class="date begin" value="' + dateStr + '"></div>'
      + '<input type="checkbox" class="use-end-stamp" checked>'
      + '<div class="parts-set stamp"><input type="time" class="time end" step="0.01" value="' + timeStr + '"><input type="date" class="date end" value="' + dateStr + '"></div>'
      ;
    return FilterUtils.getCommonHtml('time', 'from time range', conditionType, contentHtml);
  },

  getTopicsFilterHtml: function (conditionType, listist) {
    var contentHtml = '<select class="topic-value" type="select">';
    _.each(listist, function (item, index) {
      contentHtml += '<option value="' + item + '">' + item + '</option>';
    });
    contentHtml += '</select>';

    return FilterUtils.getCommonHtml('topics', 'Topics', conditionType, contentHtml);
  },

  getLocationFilterHtml: function (conditionType) {
    var contentHtml = ''
      + '<div class="parts-set textbox"><input type="text" class="location-value" placeholder="Location Text"></div>'
      + '<div class="parts-set checkbox"><input type="checkbox"><label for="">Regex</label></div>'
      ;
    return FilterUtils.getCommonHtml('location', 'Location', conditionType, contentHtml);
  },


  ////////////////////////////////////////
  // private

  getCommonHtml: function (filterType, title, conditionType, contentHtml) {
    return ''
      + '<li class="filter ' + filterType + '">'
      + '<div class="column">'
      + FilterUtils.getConditionLabelHtml(conditionType)
      + '<div class="name">'
      + '<div class="parts-set checkbox"><input type="checkbox" name="isEffective" class="isEffective" checked="checked"><label for="">' + title + '</label></div>'
      + '</div>'
      + '<div class="data">'
      + contentHtml
      + '</div>'
      + '</div>'
      + '<div class="delete">'
      + FilterUtils.getDeleteButtonHtml()
      + '</div>'
      + '</li>'
      ;
  },

  getConditionLabelHtml: function (conditionType) {
    if (conditionType === 'AND') {
      return '<span class="label and">AND</span>';
    } else {
      return '';
    }
  },

  getDeleteButtonHtml: function () {
    return '<a href class="delete-button icon"><i class="material-icons">remove_circle</i></a>';
  },

  getDateStr: function (d) {
    var year = ('0000' + d.getFullYear()).slice(-4);
    var month = ('00' + d.getMonth() + 1).slice(-2);
    var day = ('00' + d.getDate()).slice(-2);
    var str = year + '-' + month + '-' + day;
    return str;
  },

  getTimeStr: function (d) {
    var hour = ('00' + d.getHours()).slice(-2);
    var min = ('00' + d.getMinutes()).slice(-2);
    var sec = ('00' + d.getSeconds()).slice(-2);
    var msec = ('000' + d.getMilliseconds()).slice(-3);
    var str = hour + ':' + min + ':' + sec + '.' + msec;
    return str;
  },

};
