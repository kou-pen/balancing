/* Doxygen 1.9.8用。図はページの配色に依存させず、常に高コントラストの白背景で描く。 */
(function () {
  "use strict";

  document.addEventListener("DOMContentLoaded", function () {
    var figures = document.querySelectorAll("pre.mermaid");
    if (figures.length === 0) return;

    if (!window.mermaid || typeof window.mermaid.run !== "function") {
      console.error("Mermaidライブラリを読み込めませんでした。");
      return;
    }

    window.mermaid.initialize({
      startOnLoad: false,
      securityLevel: "strict",
      theme: "base",
      themeVariables: {
        background: "#ffffff",
        mainBkg: "#eff6ff",
        primaryColor: "#eff6ff",
        primaryTextColor: "#111827",
        primaryBorderColor: "#475569",
        secondaryColor: "#f1f5f9",
        secondaryTextColor: "#111827",
        secondaryBorderColor: "#64748b",
        tertiaryColor: "#ffffff",
        tertiaryTextColor: "#111827",
        textColor: "#111827",
        lineColor: "#475569",
        edgeLabelBackground: "#ffffff",
        clusterBkg: "#f8fafc",
        clusterBorder: "#94a3b8",
        actorBkg: "#eff6ff",
        actorBorder: "#475569",
        actorTextColor: "#111827",
        actorLineColor: "#64748b",
        signalColor: "#334155",
        signalTextColor: "#111827",
        labelBoxBkgColor: "#f1f5f9",
        labelBoxBorderColor: "#64748b",
        labelTextColor: "#111827",
        loopTextColor: "#111827",
        noteBkgColor: "#fff7e6",
        noteBorderColor: "#c58a27",
        noteTextColor: "#111827",
        activationBkgColor: "#dbeafe",
        activationBorderColor: "#475569",
        sequenceNumberColor: "#111827",
        fontFamily: '"Noto Sans JP", "Yu Gothic", "Meiryo", sans-serif',
        fontSize: "16px"
      },
      flowchart: { htmlLabels: false, useMaxWidth: false },
      sequence: { useMaxWidth: false, actorMargin: 70, messageMargin: 42 }
    });

    Promise.resolve(window.mermaid.run({ querySelector: "pre.mermaid" }))
      .catch(function (error) {
        console.error("Mermaid図の描画に失敗しました。", error);
      });
  });
})();
