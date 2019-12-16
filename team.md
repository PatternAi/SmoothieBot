---
layout: page
title: Team
nav_order: 3
description: A listing of project members
---
# Team
<img width="378" height="284" class="one" src="../pictures/team.jpg" alt="Team Photo"/>

# Team Members
<div class="role">
  {% assign instructors = site.staffers | where: 'role', 'Team Member' %}
  {% for staffer in instructors %}
  {{ staffer }}
  {% endfor %}
</div>
