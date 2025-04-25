document.addEventListener("DOMContentLoaded", function () {
  const versions = {
    "latest": "https://tng-blue.github.io/Isaac_ROS_WS/",
    "v1.0": "https://tng-blue.github.io/Isaac_ROS_WS/v1.0/",
    "dev": "https://tng-blue.github.io/Isaac_ROS_WS/dev/"
  };

  const container = document.createElement("div");
  container.className = "version-select";

  const select = document.createElement("select");

  for (const [label, url] of Object.entries(versions)) {
    const option = document.createElement("option");
    option.value = url;
    option.textContent = label;

    if (window.location.href.startsWith(url)) {
      option.selected = true;
    }

    select.appendChild(option);
  }

  select.addEventListener("change", () => {
    if (select.value !== window.location.href) {
      window.location.href = select.value;
    }
  });

  container.appendChild(select);
  document.body.appendChild(container);
});
