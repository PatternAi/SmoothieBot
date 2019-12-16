---
layout: page
title: Team
nav_order: 3
description: A listing of project members
---

## Team Members

<div class="role">
  {% assign instructors = site.staffers | where: 'role', 'Team Member' %}
  {% for staffer in instructors %}
  {{ staffer }}
  {% endfor %}
</div>

img.one {
  height: 50%;
  width: 50%;
}

# Team
<img  class="one" src="../pictures/team.jpg" alt="Team Photo"/>
