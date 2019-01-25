local colors = require('tests/ansicolors')
local util = require('../ur_connect/util')

function test(conditionIsMet, testDescription)
  if conditionIsMet then
    print(colors('%{green}[✓] ' .. testDescription))
  else
    print(colors('%{red}[x] ' .. testDescription))
  end
end

function table_keys(...)
  local keys = {}

  for i, t in ipairs(arg) do
    for k, v in pairs(t) do
      keys[k] = true
    end
  end

  return keys
end

function deep_equals(a, b)
  if a == nil and b == nil then
    return true
  end

  if type(a) ~= type(b) then
    return false
  end

  if type(a) == 'number' or type(a) == 'string' or type(a) == 'boolean' then
    return a == b
  elseif type(a) == 'table' then
    local all_keys = table_keys(a, b)

    for k, _ in pairs(all_keys) do
      if a[k] == nil or b[k] == nil then
        -- One of them is missing the key, so they cannot be equal
        return false
      end

      if not deep_equals(a[k], b[k]) then
        -- One of them has a different value for the key
        return false
      end
    end

    return true
  else
    error('Do not know how to compare type "' .. type(a) .. '"')
  end
end

return function()
  -- file_exists
  test(util.file_exists('tests/assets/a_file'), 'file_exists reports existing file exists')
  test(not util.file_exists('tests/assets/not_a_file'), 'file_exists reports non-existent file does not exist')

  -- read_file
  test(util.read_file('tests/assets/a_file') == 'some content', 'read_file reads correct data from file')

  -- is_byte
  test(util.is_byte(128) and util.is_byte(0) and util.is_byte(0xff), 'is_byte returns true for byte values')
  test((not util.is_byte(-5)) and (not util.is_byte(270)), 'is_byte returns false for non-byte values')

  -- int32_to_bytes and bytes_to_int32
  function _()
    local testInt32 = 0x7bcdef01
    local expected = { 0x7b, 0xcd, 0xef, 0x01 }

    local bytes = util.int32_to_bytes(testInt32)
    test(#bytes == 4 and deep_equals(bytes, expected), 'int32_to_bytes')

    local int32 = bytes_to_int32(unpack(bytes))
    test(int32 == testInt32, 'bytes_to_int32')
  end
  _()

  -- string_to_bytes and bytes_to_string
  function _()
    local testStr = 'abc'
    local expected = { 97, 98, 99 }

    local bytes = util.string_to_bytes(testStr)
    test(deep_equals(bytes, expected), 'string_to_bytes')

    local str = bytes_to_string(bytes)
    test(str == testStr, 'bytes_to_string')
  end
  _()

  -- is_valid_ip_address
  test(util.is_valid_ip_address('128.231.57.5'), 'is_valid_ip_address returns true for valid IP')
  test(util.is_valid_ip_address('0.0.0.0'), 'is_valid_ip_address returns true for valid IP address (zeros)')
  test(not util.is_valid_ip_address('random bad'), 'is_valid_ip_address returns false for misc text')
  test(not util.is_valid_ip_address('128.5.1'), 'is_valid_ip_address returns false for IP address with too few values')
end
