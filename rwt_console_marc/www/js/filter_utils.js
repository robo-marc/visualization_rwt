var FilterUtils = {

  ////////////////////////////////////////
  // public

  // equals to css class
  FILTER_TYPE: {
    MESSAGE: 'message',
    SEVERITY: 'severities',
    NODE: 'node',
    STAMP: 'time',
    TOPICS: 'topics',
    LOCATION: 'location',
  },

  isMatchedToFilterGroup: function (item, group) {
    var isAnyFilterEffective = false;
    var isAllFilterMatched = true;

    for (var i = 0, filterCount = group.length; i < filterCount; i++) {
      var filter = group[i]; // one filter

      if (!filter.isEnabled) {
        // filter not enabled
        continue;
      }

      var isMatched = false;
      var filterValue = filter.value;
      switch (filter.filterType) {
        case FilterUtils.FILTER_TYPE.MESSAGE:
          if (filterValue.message + '' === '') {
            // filter condition is empty
            continue;
          }

          isAnyFilterEffective = true;
          if (filterValue.regex) {
            isMatched = filterValue.regex.test(item.Message);
          } else {
            isMatched = (item.Message.indexOf(filterValue.message) !== -1);
          }
          break;

        case FilterUtils.FILTER_TYPE.SEVERITY:
          if (filterValue === 0) {
            // filter condition is empty
            continue;
          }

          isAnyFilterEffective = true;
          isMatched = ((item.SeverityNumber & filterValue) === item.SeverityNumber);
          break;

        case FilterUtils.FILTER_TYPE.NODE:
          if (filterValue + '' === '') {
            // filter condition is empty
            continue;
          }

          isAnyFilterEffective = true;
          isMatched = (item.Node === filterValue);
          break;

        case FilterUtils.FILTER_TYPE.STAMP:
          if (filterValue.beginDate === undefined
            && filterValue.endDate === undefined) {
            // filter condition is empty
            continue;
          }

          isAnyFilterEffective = true;
          var itemStamp = item.MilliSec;
          if (filterValue.useEndTime) {
            isMatched = (filterValue.beginDate <= itemStamp) && (itemStamp <= filterValue.endDate);
          } else {
            isMatched = (filterValue.beginDate <= itemStamp);
          }
          break;

        case FilterUtils.FILTER_TYPE.TOPICS:
          if (filterValue + '' === '') {
            // filter condition is empty
            continue;
          }

          isAnyFilterEffective = true;
          isMatched = (item.Topics.indexOf(filterValue) !== -1);
          break;

        case FilterUtils.FILTER_TYPE.LOCATION:
          if (filterValue.location + '' === '') {
            // filter condition is empty
            continue;
          }

          isAnyFilterEffective = true;
          if (filterValue.regex) {
            isMatched = filterValue.regex.test(item.Location);
          } else {
            isMatched = (item.Location.indexOf(filterValue.location) !== -1);
          }
          break;

      } // end switch
      isAllFilterMatched &= isMatched;
    } // end for

    return { isEffective: isAnyFilterEffective, isMatched: isAllFilterMatched };
  },

  isMatchedToAnyFilterGroup: function (item, filterGroupList, useHighlight) {
    if (useHighlight) {
      item['_matched'] = undefined;
    }

    var isAnyGroupEffective = false;
    var isAnyGroupMatched = false;
    for (var i = 0, groupCount = filterGroupList.length; i < groupCount; i++) {
      var group = filterGroupList[i]; // one filter group

      var groupResult = FilterUtils.isMatchedToFilterGroup(item, group);
      if (groupResult.isEffective) {
        isAnyGroupEffective = true;
        isAnyGroupMatched |= groupResult.isMatched;
      }
    }

    if (useHighlight && isAnyGroupEffective) {
      // Converts isAnyGroupMatched to Boolean because it was converted to Number by logical operation
      item['_matched'] = (isAnyGroupMatched ? true : false);
    }

    return isAnyGroupMatched;
  },

  getMessageFilterHtml: function (conditionType) {
    var contentHtml = ''
      + '<div class="parts-set textbox"><input type="text" class="message-value"placeholder="Message Text"></div>'
      + '<div class="parts-set checkbox"><label><input type="checkbox" class="regex">Regex</label></div>'
      ;
    return FilterUtils.getCommonHtml(FilterUtils.FILTER_TYPE.MESSAGE, 'Message', conditionType, contentHtml);
  },

  getSeverityFilterHtml: function (conditionType) {
    var contentHtml = ''
      + '<ul class="chips">'
      + '<li><a href class="severities-select-button" data-value="1">Debug</a></li>'
      + '<li><a href class="severities-select-button" data-value="2">Info</a></li>'
      + '<li><a href class="severities-select-button" data-value="4">Warn</a></li>'
      + '<li><a href class="severities-select-button" data-value="8">Error</a></li>'
      + '<li><a href class="severities-select-button" data-value="16">Fatal</a></li>'
      + '</ul>'
      ;
    return FilterUtils.getCommonHtml(FilterUtils.FILTER_TYPE.SEVERITY, 'Severities', conditionType, contentHtml);
  },

  getNodesFilterHtml: function (conditionType, list) {
    var contentHtml = '<select class="node-value" type="select">';
    _.each(list, function (item, index) {
      contentHtml += '<option value="' + item + '">' + item + '</option>';
    });
    contentHtml += '</select>';

    return FilterUtils.getCommonHtml(FilterUtils.FILTER_TYPE.NODE, 'Node', conditionType, contentHtml);
  },

  getStampFilterHtml: function (conditionType) {
    var now = new Date();
    var dateStr = FilterUtils.getDateStr(now);
    var timeStr = FilterUtils.getTimeStr(now);
    var contentHtml = ''
      + '<div class="parts-set stamp"><input type="time" class="time begin" step="0.001" value="' + timeStr + '"><input type="date" class="date begin" value="' + dateStr + '"></div>'
      + '<input type="checkbox" class="use-end-stamp" checked>'
      + '<div class="parts-set stamp"><input type="time" class="time end" step="0.001" value="' + timeStr + '"><input type="date" class="date end" value="' + dateStr + '"></div>'
      ;
    return FilterUtils.getCommonHtml(FilterUtils.FILTER_TYPE.STAMP, 'from time range', conditionType, contentHtml);
  },

  getTopicsFilterHtml: function (conditionType, listist) {
    var contentHtml = '<select class="topic-value" type="select">';
    _.each(listist, function (item, index) {
      contentHtml += '<option value="' + item + '">' + item + '</option>';
    });
    contentHtml += '</select>';

    return FilterUtils.getCommonHtml(FilterUtils.FILTER_TYPE.TOPICS, 'Topics', conditionType, contentHtml);
  },

  getLocationFilterHtml: function (conditionType) {
    var contentHtml = ''
      + '<div class="parts-set textbox"><input type="text" class="location-value" placeholder="Location Text"></div>'
      + '<div class="parts-set checkbox"><label><input type="checkbox" class="regex">Regex</label></div>'
      ;
    return FilterUtils.getCommonHtml(FilterUtils.FILTER_TYPE.LOCATION, 'Location', conditionType, contentHtml);
  },


  ////////////////////////////////////////
  // private

  getCommonHtml: function (filterType, title, conditionType, contentHtml) {
    return ''
      + '<li class="filter ' + filterType + '">'
      + '<div class="column">'
      + FilterUtils.getConditionLabelHtml(conditionType)
      + '<div class="name">'
      + '<div class="parts-set checkbox"><label><input type="checkbox" name="isEffective" class="isEffective" checked="checked" />' + title + '</label></div>'
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
