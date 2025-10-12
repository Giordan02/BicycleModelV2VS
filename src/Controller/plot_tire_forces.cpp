#include "src/Controller/plot_tire_forces.h"
#include "src/Model/qcustomplot.h"


void configure_plot(QCustomPlot *tireplot, QVector<double> x, QVector<double> y, QString xLabel, QString yLabel){
    // Configure plot displaying and setting longitudinal and lateral values for the axis 
    // --- Basic Plot Configuration ---
    QObject::disconnect(tireplot, &QCustomPlot::mouseMove, nullptr, nullptr);
    tireplot->clearGraphs(); 
    tireplot->clearItems();
    tireplot->addGraph();
    tireplot->graph(0)->setData(x, y);

    // Set a pen for the graph
    QPen graphPen;
    graphPen.setColor(QColor(50, 120, 220));
    graphPen.setWidthF(2);
    tireplot->graph(0)->setPen(graphPen);

    tireplot->xAxis->setLabel(xLabel);
    tireplot->yAxis->setLabel(yLabel);
    tireplot->xAxis->setRange(x.first(), x.last());
    double yMin = *std::min_element(y.constBegin(), y.constEnd());
    double yMax = *std::max_element(y.constBegin(), y.constEnd());
    double margin = 0.1 * std::max(std::abs(yMin), std::abs(yMax)); // On the y axis, the standard is to show 10% more of the maximum and minimum heights
    tireplot->yAxis->setRange(yMin - margin, yMax + margin);

    // Permits the zoom of the graph and displaying the data points for where the cursor is
    tireplot->setInteraction(QCP::iRangeDrag, true);
    tireplot->setInteraction(QCP::iRangeZoom, true);
    tireplot->setInteraction(QCP::iSelectPlottables, true);

    // --- Interactive Tracer Implementation ---

    // 1. Setup the Tracer 
    QCPItemTracer *tracer = new QCPItemTracer(tireplot);
    tracer->setGraph(tireplot->graph(0));
    tracer->setInterpolating(false);
    tracer->setStyle(QCPItemTracer::tsCircle);
    tracer->setPen(QPen(Qt::red, 1.5));
    tracer->setBrush(Qt::red);
    tracer->setSize(7);

    // 2. Setup the Label 
    QCPItemText *label = new QCPItemText(tireplot);
    label->setLayer("overlay");
    label->setPadding(QMargins(5, 5, 5, 5));
    label->setBrush(QBrush(QColor(240, 240, 240, 220)));
    label->setPen(QPen(Qt::gray));
    label->position->setParentAnchor(tracer->position);
    label->setFont(QFont("sans", 9));
    label->setText("");

    // Hide tracer and label initially
    tracer->setVisible(false);
    label->setVisible(false);

    // 3. Connect mouse movement to update the tracer
    QObject::connect(tireplot, &QCustomPlot::mouseMove, [=](QMouseEvent *event) {
        // Ensure there is data to trace
        if (tireplot->graph(0)->data()->isEmpty()) return;

        // Find the data point closest to the cursor
        double closestKey;
        double closestValue;
        double minDistance = std::numeric_limits<double>::max();
        
        QCPGraphDataContainer::const_iterator begin = tireplot->graph(0)->data()->constBegin();
        QCPGraphDataContainer::const_iterator end = tireplot->graph(0)->data()->constEnd();
        
        for (auto it = begin; it != end; ++it) {
            // Use pixel distance to find the closest point
            QPointF pointPixel = tireplot->graph(0)->coordsToPixels(it->key, it->value);
            double dist = QLineF(pointPixel, event->pos()).length();

            if (dist < minDistance) {
                minDistance = dist;
                closestKey = it->key;
                closestValue = it->value;
            }
        }

        // Update tracer and label with the found data
        tracer->setGraphKey(closestKey);
        label->setText(QString("X: %1\nY: %2").arg(closestKey, 0, 'f', 2).arg(closestValue, 0, 'f', 2));

        // Get the center of the visible axis ranges to align relatively to them
        double xCenter = tireplot->xAxis->range().center();
        double yCenter = tireplot->yAxis->range().center();

        // Set label horizontal and vertical alignment
        Qt::Alignment hAlign = (closestKey < xCenter) ? Qt::AlignLeft : Qt::AlignRight;
        Qt::Alignment vAlign = (closestValue > yCenter) ? Qt::AlignTop : Qt::AlignBottom;
        label->setPositionAlignment(hAlign | vAlign);
        tracer->setVisible(true);
        label->setVisible(true);
        
        tireplot->replot();
    });

    // Initial replot to draw the graph
    tireplot->replot();
}