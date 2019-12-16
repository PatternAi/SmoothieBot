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

# Team
<img width=2016 height=1512 class="one" src="../pictures/team.jpg" alt="Team Photo"/>
