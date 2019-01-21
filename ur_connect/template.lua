--- Mustache-style templating. Replaces {{foo}} with a provided value of foo.
-- @param template The template string.
-- @param values A table of values to insert into the template.
function render(template, values)
  local rendered = template

  for k, v in pairs(values) do
    rendered, count = string.gsub(rendered, '{{' .. k .. '}}', '' .. v)

    if count == 0 then
      error('Template value "' .. k .. '"not present in template given')
    end
  end

  return rendered
end

return {
  render = render,
}
